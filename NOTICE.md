# NOTICE — 第三方素材与生成代码声明

本工程的主体代码以 **MIT 许可证**发布（见 `LICENSE`）。
以下内容来自第三方，遵循各自的许可证，**不受 MIT 覆盖**。

---

## 1. 12×12 中文点阵字库（GPL v2 + 字体嵌入例外）

| 项目 | 内容 |
| --- | --- |
| 文件 | `hardware/wqy12_common.h`<br>`pc/font/wqy12_common.bin`<br>`pc/font/wqy12_gb2312.bin`<br>`pc/font/wqy12_index.txt` |
| 来源 | 文泉驿点阵宋体（WenQuanYi Bitmap Song） |
| 许可证 | **GNU GPL v2 + 字体嵌入例外（Font Embedding Exception）** |

上述文件中的点阵数据均由该字库经 `pc/font/tools/` 的脚本生成，属于**衍生作品**，
因此这部分数据遵循 GPL v2，而不是 MIT。GPL v2 全文见
<https://www.gnu.org/licenses/old-licenses/gpl-2.0.html>。

⚠ 原始 BDF 字库（`wenquanyi_10pt.bdf`，约 4.4 MB）**未包含在本仓库中**，
需要重建字库时请自行从文泉驿官网下载，放到 `pc/font/_src/` 下。

若你需要一个纯 MIT 的版本，把该文件替换成自己设计或来自 OFL 授权字体的点阵即可，
其余代码不受影响。

---

## 2. TI SysConfig 生成代码

| 项目 | 内容 |
| --- | --- |
| 文件 | `Debug/ti_msp_dl_config.c`、`Debug/ti_msp_dl_config.h` |
| 来源 | 由 TI SysConfig 依据 `gpio_toggle_output.syscfg` 自动生成 |
| 许可证 | Texas Instruments MSPM0 SDK Software License Agreement |

这些文件是 SysConfig 工具的输出，**可重新生成**。本仓库不包含 TI SDK 本体，
构建时需要在本机安装 MSPM0 SDK 并设置路径。

---

## 3. CMSIS

| 项目 | 内容 |
| --- | --- |
| 用途 | Cortex-M 内核头文件（`core_cm0plus.h` 等） |
| 来源 | ARM CMSIS |
| 许可证 | Apache License 2.0 |

本仓库不包含 CMSIS 源码，构建时从 MSPM0 SDK 的 `third_party` 目录引用。

---

## 关于已移除的代码

早期版本曾包含 `hardware/IMU660RB/`（STMicroelectronics LSM6DSR 驱动与
xioTechnologies Fusion 姿态解算库）。这部分代码**从未被本工程引用**，
也未链接进最终固件，已在公开前移除，其许可证与本工程无关。
