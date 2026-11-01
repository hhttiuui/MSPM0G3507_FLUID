# 硬件细节：MATHACL、面板映射、封装

## MATHACL 硬件除法器

MSPM0G3507 内置 **MATHACL 数学加速器**（TI 官方描述为 “Trigonometric math
accelerator”），提供定点除法、开方、sin/cos、四象限反正切、乘加等运算。
本工程用它替换了 Q24 除法。

**开启方式（三步，不需要配任何参数）**：

1. `gpio_toggle_output.syscfg` 里加模块：

   ```javascript
   const MATHACL = scripting.addModule("/ti/driverlib/MATHACL");
   ```
2. SysConfig 生成的 `SYSCFG_DL_init()` 里会出现两行 —— **少了第二行会静默失效**
   （寄存器写得进，但结果永远是 0，很难查）：

   ```c
   DL_MathACL_reset(MATHACL);
   DL_MathACL_enablePower(MATHACL);
   ```
3. 直接读写寄存器。调用序列**必须**按这个顺序：

   ```c
   MATHACL->CTL = 操作码 | (Q格式 << 8) | (1 << 5);  /* bit5 = 立即触发 */
   MATHACL->OP2 = 第二个操作数;
   MATHACL->OP1 = 第一个操作数;   /* 写 OP1 就是发令，必须最后写 */
   结果 = MATHACL->RES1;          /* 读 RES1 时硬件会 hold 总线直到算完 */
   ```

**没有轮询循环、没有中断**：写 `OP1` 触发，读 `RES1` 取结果，靠 AHB 等待自动同步。

本工程用到的操作码（Q24 时的 CTL 值）：

| 操作 | 操作码 | CTL                                   |
| ---- | ------ | ------------------------------------- |
| DIV  | 4      | `0x1824` = `4 \| (24<<8) \| (1<<5)` |

验证方法：用 `tiarmclang -S` 输出汇编，常量池里应该能看到 `.long 6180`（即
`0x1824`）和 `_IQdiv` 的 4 条寄存器访问。

**`_IQmpy` 故意没换**。FOC 工程用 `-D_IQ_MPY_MATHACL_INLINE` 把乘法也交给
MATHACL，那是因为它用 **IQ27**：32×32 乘法要 64 位中间结果 → 汇编里是
`__aeabi_lmul`（约 30~60 周期 + 函数调用）。而本工程的 **Q24 乘法已经内联成
4 次 16×16 乘法**（约 30 周期、无调用），再换成 MATHACL 要付出 4 次外设总线
访问，收益存疑甚至可能变慢。

> ⚠️ **MATHACL 不可重入** —— 只有一组 `CTL/OP1/OP2/RES1`。目前只有主循环用它，
> 安全；以后如果中断里也要用，必须加互斥。


## 流体模拟 → LED 点阵（hardware/fluid_demo.c）

`fluid.c` 是纯算法（15 列 × 16 行可视区，和面板 240 颗一一对应），本文件只是胶水层：

| 部分 | 做法 |
| ---- | ---- |
| 驱动 | **真实 IMU**：加速度计读数按频率拆成"慢 = 重力/倾角"和"快 = 摇晃/惯性"两条通路再相加（原理与调参见 `fluid_demo.c` 的《重力 / 摇晃：双通路分解》）。IMU 初始化失败就自动退回**假重力**（方向每 2 s 换一次：下→左→上→右） |
| 渲染 | **灰度**：`particleDensity[]` → 亮度 → `LedMatrix_SetCellLevel(x, y, lv)`（内核亮、表层暗），最后一帧一次推屏。二值模式自动退回阈值 + 滞回 |
| 观测量 | 全在 `dbg.fluid`：`ax10/ay10`（喂给流体的总量）、`ax_slow10/ay_slow10`（重力分量）、`ax_fast10/ay_fast10`（摇晃分量，已含增益）、`shake_pk10`（窗口内摇晃峰值）、`sim_ms_x10`（单步物理耗时 ×10）、`fps`、`frames`、`tilt`、`iq_ok`、`imu_ok`/`imu_use` |
| 数学层自检 | 开机先验 `_IQmpy/_IQdiv/_IQsqrt`；失败就把**整屏点亮**停住（结果记在 `dbg.fluid.iq_ok`）|

参数在文件顶部：`DEMO_DT`（时间步长 0.032）、`DEMO_GRID_ITER`（压力迭代 12 次）、
`DEMO_TILT_MS`（退回假重力后的换向周期）、`FLUID_IMU_LPF_ALPHA`（慢通路/重力平滑）、
`FLUID_SHAKE_ALPHA` + `FLUID_SHAKE_GAIN`（快通路/摇晃灵敏度）、`FLUID_SHAKE_DEADBAND`
（摇晃死区，挡倾斜态噪声）、`FLUID_ACC_CLAMP`（合成后限幅）、`FLUID_USE_IMU`（0 = 强制假重力）。

**摇晃灵敏度怎么调**：只动 `FLUID_SHAKE_ALPHA` / `FLUID_SHAKE_GAIN`，**不要**为了摇晃去调大
`FLUID_IMU_LPF_ALPHA` —— 那个是重力通路的平滑，调大只会让静止的液体跟着噪声抖。

## LED 点阵（HT16D33A）综合演示

只有**一个**模式，上电就循环播放（实现 [hardware/led_show.c](hardware/led_show.c)）：

| 段 | 时长 | 内容 |
| -- | ---- | ---- |
| ① 全亮 | 500 ms | 面板上 240 颗（走映射，不碰 CA19 那些没有 LED 的槽） |
| ② 全黑 | 500 ms | 顺便当残影检查 |
| ③ 逐位游走 | 240 × 20 ms | **按映射表**的 (行,列) 光栅顺序走一格，干净地从左到右、从上到下 |
| ④ 图案 | 2000 ms | 外框 + 主对角线 + 左上角 2×2 实心块（顺便当方向标定图案） |

一轮 ≈ 7.8 秒，然后循环。各段时长是 [hardware/led_show.h](hardware/led_show.h) 顶部的
`LED_SHOW_*_MS` 宏，改完重编即可。

**给以后串口指令预留的接口**（同一个头文件）：

```c
LedShow_Start();        /* 从头开始播放。只写一个标志位，从串口中断里调也安全 */
LedShow_Stop();         /* 立即停 */
LedShow_IsActive();     /* 1 = 正在播放 */
LedShow_Task();         /* ⚠ 主循环里每 ≥1ms 调一次（里面有 SPI，别放中断里）*/
LedShow_Run();          /* 初始化 + 死循环，给 main() 当独立演示程序用 */
```

接进别的应用（比如流体）时用前 4 个：串口命令里调 `LedShow_Start()`，主循环里调
`LedShow_Task()`，并用 `LedShow_IsActive()` 判断要不要让出显示权。

**观测量**：`dbg.led.show_stage`（0 没在播 / 1 全亮 / 2 全黑 / 3 游走 / 4 图案）、
`dbg.led.step`（游走格号 0..239）、`dbg.led.xfer`（每帧 +34）、`dbg.led.timeout`（应恒 0）。

## 面板映射（hardware/led_matrix.c）

手册只给了"逻辑通道 → RAM 位"的公式，**这块面板的引脚和逻辑通道之间还差两次旋转**，
全部收在 [hardware/led_matrix.c](hardware/led_matrix.c) 里，上层只认 (列, 行)：

| 事实 | 内容 |
| ---- | ---- |
| 引脚编号 | 16×16 的有效脚 = S1..S17 = `CA1..CA8, CA10, CA12..CA18, CA19`（CA9/CA11 高阻，跳过） |
| 面板走线 | **阶梯形**：第 r 行的阴极一律是 S(c)；阳极多为 S(17−r)，但 `c > 16−r` 那一小段改用 S(16−r)（切点每下一行左移 1 格） |
| 扫描通道 | 通道与阳极引脚一一对应：`n = a`（即通道 1 ↔ CA1、…、通道 16 ↔ CA18）<br>⚠ 一开始误以为是反序轮转，现象是光栅在第 15 步从"第一行最后一颗"跳到"最后一行最后一列"；**修正后已实测通过** |
| 数据通道 | 手册规定本相位的脚不参与数据，所以 `m = c − (c > a ? 1 : 0)` |
| 最终 | `addr = 2×(n−1) + (m−1)/8`、`bit = (m−1)%8` |

因此：**面板 16 列 × 15 行 = 240 颗 ↔ 240 个 RAM 位**（脚本核对过是双射，无重复无缺漏）；
**16 个用不到的 RAM 位正好是每一相的 m=16 → 全部指向 CA19**，所以 CA19 悬空是正确的。
每一相恰好 15 颗 × 16 相 = 240 ✓。**整个映射层已用实测光栅验证通过。**

> ⚠️ 按原始 RAM 顺序逐位走（比如不经过映射直接写位）看起来会"斜着走"，那是阶梯形
> 走线造成的，**不是 bug**；走映射层才是干净的光栅。

**方向开关**（面板 16 列 × 15 行 与 流体网格 15 列 × 16 行 是转置关系）：
看综合演示的**图案段**（外框 + 对角线 + 左上角 2×2 块）调
[hardware/led_matrix.h](hardware/led_matrix.h) 里的 `LED_ORIENT_SWAP_XY / FLIP_X / FLIP_Y`，
目标是把实心块摆到设备的左上角。

```c
LedMatrix_Init();                      /* 初始化芯片（含清屏） */
LedMatrix_Clear();                     /* 清本地帧缓冲 */
LedMatrix_SetCellLevel(x, y, level);   /* 灰度：按流体网格坐标写 0~255 亮度 */
LedMatrix_SetCell(x, y);               /* 二值：写满亮度 */
LedMatrix_FillAll(1u);                 /* 全亮（走映射，不碰没有 LED 的槽）*/
LedMatrix_Flush();                     /* 整帧推送：256 B 分 8 个 burst，约 1.2 ms */
```

> 灰度模式（`HT16D33A_USE_BINARY_MODE = 0`）下 DDRAM 是 **256 字节**，
> 一点一字节；二值模式只有 32 字节，推屏更快但液面是硬边。



封装是 **LQFP-48 (PT)**，订货型号 **MSPM0G3507SPTR**。

PT 与 VQFN-48(RGZ) 的引脚定义**完全一致**（手册里两者共用同一列脚号），
所以换封装后 SysConfig 生成的文件逐字节不变，**应用代码零改动**。
唯一区别是散热（RθJA 69.2 vs 30.1 °C/W）和焊接难度 —— 本工程 MCU 功耗仅约 50 mW，
温升约 3.5 °C，用 LQFP 更好焊好查。

关键的器件事实：**48 脚的 44 个 GPIO 是 64 脚的完全子集**，所以只要引脚选在
两边都有的那些上，换封装时应用代码一行都不用改 —— 代码里全是 SysConfig 的
逻辑名（`SPI_LSM6DS3TRC_INST`、`GPIO_LSM6DS3TRC_PORT`……），端口号和引脚号
由自动生成的 `ti_msp_dl_config.h` 决定。

48 脚上**不存在**的 16 个引脚（永远不要用）：

```text
PA29 PA30 PB0  PB1  PB4  PB5  PB10 PB11
PB12 PB13 PB21 PB22 PB23 PB25 PB26 PB27
```

当前实际引脚分布见 [README 的「硬件接线」一节](../README.md)。

> 曾经为了用 `DOUTSET31_0` 一次写一整相位而设计的「PA0–PA15 十六路 LED 直驱」
> 方案**已经废弃** —— 显示改成了 SPI 接口的 HT16D33A 点阵，`GPIO_LED_SHOW`
> 也不在 `.syscfg` 里了。相应的引脚搬迁（OLED/LED/UART 搬到 PORTB）不再必要。

> 换板子时唯一不能省的硬件项：**VCORE 的外部去耦电容**。

