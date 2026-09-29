# MSPM0G3507 四轮小车

本工程运行在 MSPM0G3507 上，包含四路电机、两路编码器、四个按键、
SSD1306 OLED 和 IMU660RB。当前速度环为已经完成架空、负载和地面验证的
正式 V2 版本，测试串口、在线调参和道路批测代码已移除。

## 调度结构

`TIMER_0` 每 1ms 进入一次中断，并循环执行五个调度槽：

| 槽位 | 功能 |
| --- | --- |
| 0 | 编码器任务和 IMU 原始数据读取 |
| 1 | IMU 姿态解算 |
| 2 | 有新编码器样本时执行 20ms 速度环 |
| 3 | 预留给 IMU 航向环或循迹外环 |
| 4 | 空闲 |

四个按键每 1ms 采样和消抖。OLED 在主循环中显示循迹字节、M1/M3 编码器、
姿态角以及 KEY1~KEY4 的稳定按下时间。

## 四轮速度控制

| 车轮 | 控制方式 | 说明 |
| --- | --- | --- |
| M1 右后 | 闭环 | 使用 M1 编码器反馈 |
| M2 右前 | 跟随 | 跟随 M1 输出，正式补偿为 `+75‰` |
| M3 左后 | 闭环 | 使用 M3 编码器反馈 |
| M4 左前 | 跟随 | 跟随 M3 输出，正式补偿为 `-75‰` |

## 参数调整入口

速度环参数集中在 `hardware/speed_control_config.h`，常规调参不需要修改
`speed_control.c` 的算法实现。

| 要调整的内容 | 修改位置 | 说明 |
| --- | --- | --- |
| 运行目标速度 | `SpeedControl_SetTargets(left, right)` | 单位为每 20ms 编码器脉冲数 |
| Kp / Ki / Kd | `SPEED_PID_KP/KI/KD` | 当前 Ki 已包含 20ms 控制周期 |
| PWM 上限 | `SPEED_OUTPUT_LIMIT` | 800 表示 PWM 最大 80% |
| M4 / M2 静态补偿 | `SPEED_M4/M2_TRIM_PERMILLE` | 单位为千分比 |
| 目标斜坡与前馈 | 配置文件的“V2 高级参数” | 修改后必须重新完成标准化测试 |
| 引脚和外设 | `gpio_toggle_output.syscfg` | 建议使用 CCS SysConfig 图形界面修改 |

当前固化参数：

- `Kp = 30.0`
- `Ki = 1.5`，已包含 20ms 控制周期
- `Kd = 0.0`
- PWM 输出上限 `800/1000`
- 目标斜坡 `1 pulse / 20ms`
- 正向前馈根据 Target 8~20 标定
- 包含反向积分卸载和抗积分饱和回算

静态补偿只在左右两侧目标都为正时生效。车辆航向的动态偏差应由后续 IMU
航向外环修正，不应继续修改速度环来代替航向控制。

## 正式接口

```c
SpeedControl_Init();
SpeedControl_SetTargets(leftTarget, rightTarget);
SpeedControl_Start();

/* 每 20ms 编码器采样后调用 */
SpeedControl_Update20ms(leftActual, rightActual);

SpeedControl_Stop();
```

上电后速度环默认停止。后续菜单、按键功能或 IMU 外环应通过
`SpeedControl_SetTargets()`、`SpeedControl_Start()` 和
`SpeedControl_Stop()` 控制车辆。

## 主要引脚

| 功能 | 引脚 |
| --- | --- |
| KEY1 / KEY2 / KEY3 / KEY4 | PA2 / PB1 / PB17 / PB20 |
| OLED SDA / SCL | PA16 / PA15 |
| M1 编码器 A / B | PA13 / PA12 |
| M3 编码器 A / B | PB5 / PB4 |
| IMU SPI SCLK / MOSI / MISO / CS | PA17 / PB22 / PB14 / PB0 |
| 预留循迹 UART TX / RX | PA8 / PA9 |

PA10、PA11 原先用于 VOFA 调试串口，正式版本中已经释放。

## 目录

| 路径 | 内容 |
| --- | --- |
| `gpio_toggle_output.c` | 初始化、1ms 调度、中断和 OLED 主循环 |
| `gpio_toggle_output.syscfg` | 外设与引脚配置 |
| `hardware/speed_control_config.h` | 速度环统一参数调整入口 |
| `hardware/speed_control.c` | 正式 V2 速度环 |
| `hardware/motor.c` | 四路 PWM 和方向控制 |
| `hardware/encoder.c` | M1/M3 编码器采样 |
| `hardware/key.c` | 四按键采样、消抖和按下计时 |
| `hardware/IMU660RB/` | IMU 驱动与姿态解算 |
| `test_results/` | 已完成测试的原始数据归档，不参与固件编译 |

## 编译

在 CCS 中执行 Build，或运行：

```powershell
C:\ti\ccs2041\ccs\utils\bin\gmake.exe -C Debug -j 16 all
```
