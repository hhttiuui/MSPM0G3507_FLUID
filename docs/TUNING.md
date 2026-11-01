# 调参与标定

## 可调参数

| 想改什么                               | 改哪里                                                        |
| -------------------------------------- | ------------------------------------------------------------- |
| 粒子数、网格尺寸、粒子半径、弹性、间距 | `hardware/fluid.h`                                          |
| 时间步长`dt`、压力迭代次数           | `hardware/fluid_demo.c` 的 `DEMO_DT` / `DEMO_GRID_ITER` |
| 加速度计轴向映射、符号                 | `hardware/app.c` 顶部的 `APP_IMU_AX_SRC` / `_SIGN` 等 |
| 重力平滑 / 摇晃灵敏度 / 摇晃死区       | 同上的 `APP_IMU_LPF_ALPHA`、`APP_SHAKE_*` 宏 |
| **显示方向（镜像 / 转置）**            | `hardware/led_matrix.h` 的 `LED_ORIENT_*` |
| `dbg.fluid` 统计窗口帧数               | `hardware/fluid_demo.c` 的 `DEMO_STAT_FRAMES`             |
| 引脚和外设                             | `gpio_toggle_output.syscfg`（用 CCS SysConfig 改）          |

## 换板子后要重标定加速度计轴向

本板实测结论（已写死在 `hardware/app.c`）：

```c
#define APP_IMU_AX_SRC      1     /* 屏幕 x ← AY */
#define APP_IMU_AY_SRC      0     /* 屏幕 y ← AX */
#define APP_IMU_AX_SIGN     (+1)
#define APP_IMU_AY_SIGN     (-1)
```

即 **两轴对调**，x 取正、y 取负。

> ⚠️ **`AX_SIGN` 的符号和显示方向是绑死的。**
> 这套映射是用"摆三个姿势、看液体落在哪边"标出来的，参照物是屏幕。
> 2026-10-24 发现屏幕是**左右镜像**的（文字暴露的，液体看不出来），
> 修 `LED_ORIENT_FLIP_Y` 的同时把 `AX_SIGN` 从 -1 翻成 +1，抵消 x 轴反向。
> 只改一个 → 文字或液体必有一个是反的。详见 `app.c` 里那段注释。

**三点标定法**：把屏幕按"正着放 → 顺时针 90° → 顺时针 180°"各停一次，
记下液体落在哪边，和"应该落在哪边"（下 → 右 → 上）对比，即可解出这 4 个宏。

口诀：

- 液体流向与倾斜方向相反 → 该轴 `SIGN` 取反
- 液体左右流但你是上下倾斜 → 两个 `SRC` 对调

验证：`dbg.fluid.ax10 / ay10` 正着放时应为 `(0, +98)`，即"向下"。

## 显示方向（`LED_ORIENT_*`）

```c
#define LED_ORIENT_SWAP_XY  1u      /* 面板 16 宽 ↔ 应用 16 高（转置，必须为 1）*/
#define LED_ORIENT_FLIP_X   0u      /* 1 = 上下镜像 */
#define LED_ORIENT_FLIP_Y   1u      /* 1 = 左右镜像 */
```

**这两个名字是按面板坐标命名的，`SWAP_XY=1` 时和屏幕方向正好相反** ——
`FLIP_X` 翻的是面板列（= 应用 y，屏幕上表现为上下），`FLIP_Y` 翻的是面板行
（= 应用 x，屏幕上表现为左右）。按名字去调会调到另一根轴上去。

| 现象 | 改哪个 |
| --- | --- |
| 文字左右镜像（像照镜子）| `FLIP_Y` |
| 文字上下颠倒 | `FLIP_X` |
| 图案长宽比对不上 | `SWAP_XY`（本板必须是 1，否则应用 y=16 超出面板 15 行）|

判断基准：发 `pat`，**2×2 实心块应该落在左上角**。

⚠️ 改 `FLIP_X/FLIP_Y` 之后**必须同步改 `app.c` 里对应的 `APP_IMU_*_SIGN`**，
否则液体流向会跟着镜像过去。

