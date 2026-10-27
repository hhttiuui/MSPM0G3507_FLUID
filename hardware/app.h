/*
 * app.h —— 场景框架：主循环 / 输入层 / 场景切换
 *
 * ==================== 为什么要有这一层 ====================
 *
 * 原来 fluid_demo.c 是"一个场景写死在一个死循环里"。想再加静态图、文字滚动、
 * 贪吃蛇……如果继续用 `if (mode == X)` 堆分支，几个模式会互相污染：每个模式
 * 都得判断 s_dispLock、s_paused、IMU 到底通没通，越加越乱。
 *
 * 这一层只做四件事：
 *   ① **输入**：IMU 双通路 → app_in_t。所有场景共用一份，不再各自读 IMU。
 *   ② **分发**：每帧只调**一个**场景的 tick。
 *   ③ **切换**：App_Goto() 负责 exit → 清屏 → enter。
 *   ④ **命令**：全局命令（遥测、亮度）在这里处理，其余转给当前场景的 cmd。
 *
 * ==================== 铁律 ====================
 *
 * **同一时刻只有一个场景能写 DDRAM。**
 *   两个场景同时写会得到两幅画叠在一起的花屏，而且这种 bug 只在切换瞬间出现，
 *   极难复现。切换时强制 Clear + Flush 就是为了把"上一幅画"擦干净 ——
 *   DDRAM 是持久的，不清的话没人擦过的地方会一直亮着。
 *
 * ==================== 加一个新场景 ====================
 *
 *   1. 写 XXXScene_Enter / Tick / Exit / Cmd 四个函数（声明放 scenes.h）
 *   2. 在 app.c 的 s_scenes[] 里加一行
 *   3. 改 Debug/ 的构建列表（新增 .c 才需要）
 *
 * ⚠ 不要用 `__attribute__((section("scenes")))` + 链接器自动收集来自动注册。
 *   本工程的 device_linker.cmd 每次 make 都被 SysConfig 重新生成
 *   （subdir_rules.mk 明确依赖 .syscfg），手加的 SECTION 下次构建就没了。
 *   静态 const 数组最稳。
 */
#ifndef __APP_H
#define __APP_H

#include <stdint.h>
#include "fluid_iqmath.h"       /* _iq */

/* ==================== 输入包 ====================
 *
 * 每帧算一次，交给当前场景。场景**不要**自己去读 IMU —— 一次 LSM6DS3TRC_ReadAll()
 * 加双通路滤波是共享成本，而且摇晃分量必须连续滤波，两个场景各读一次会互相打乱。
 */
typedef struct {
    _iq      ax, ay;            /* 合成后总加速度（模拟单位，已限幅）—— 喂物理用 */
    _iq      ax_slow, ay_slow;  /* 慢通路：重力 / 倾角 */
    _iq      shx, shy;          /* 快通路 × 增益：摇晃 / 惯性 */

    /* **未滤波**的原始加速度（屏幕坐标、模拟单位）。
     * 给"想自己决定怎么滤波"的消费者用 —— 目前只有刹车检测（brake.c）：
     * 它要的是 90 ms 级的慢速低通，和场景用的双通路滤波（α=0.10 / 0.60）
     * 完全不是一回事，共用一份只会互相拖累。
     * ⚠ 没有 IMU 时这两个字段恒为 0。 */
    _iq      ax_raw, ay_raw;

    uint32_t now_ms;            /* tick_ms 快照 */
    uint16_t dt_ms;             /* 距上次 tick 的毫秒数（首帧为 0）*/

    uint8_t  key;               /* bit0..2 = USER1..3 的"按下"边沿，读后由 app 清位 */
    uint8_t  shake10;           /* 本帧摇晃幅度 ×10，取绝对值、截断到 255 */
} app_in_t;

/* 按键位（对应 key_id_t 的顺序）
 *
 * ⚠ 本板只装了 USER1(PA25) 和 USER3(PA23) 两个按键。
 *   USER2(PA24) **引脚悬空**，别拿它做任何功能 —— 悬空输入经去抖可能
 *   随机被承认成"按下"。syscfg 里给它开了内部上拉，读数稳定但没按键。
 *   宏保留着，以后焊上按键就能直接用。 */
#define APP_KEY_BIT(k)      ((uint8_t) (1u << (uint8_t) (k)))
#define APP_KEY_USER1       APP_KEY_BIT(0)
#define APP_KEY_USER2       APP_KEY_BIT(1)   /* 未装按键 */
#define APP_KEY_USER3       APP_KEY_BIT(2)

/* ==================== 场景 ====================
 *
 * 四个回调全部可以为 NULL。约束：
 *   · tick()  **必须非阻塞**：禁止 while 等硬件、禁止 mspm0_delay_ms。
 *              一旦卡住，按键和关机全失灵（只有 TIMA0 中断那条路还活着）。
 *   · tick()  **只能写 DDRAM，不能改芯片模式**（BGS 位要重新 init，会闪屏）。
 *   · enter() 必须自己初始化自己的状态。场景之间共用一块 scratch，
 *              不要假设进来时里面是干净的（可能是上一个场景留下的）。
 *   · cmd()   返回 1 = 已处理，0 = 不是我的命令（app 会填 result = 3）。
 */
typedef struct {
    void    (*enter)(void);
    void    (*tick)(const app_in_t *in);
    void    (*exit)(void);
    uint8_t (*cmd)(uint16_t id, const int32_t *arg);
} scene_t;

/* ==================== 共享 scratch ====================
 *
 * 小场景的**大块像素缓冲**都放这里，同一时刻只有一个成员有效。
 * 理由：SRAM 只剩 ~9 KB，而文字条带要 512 B、蛇身要几十字节 ——
 * 每个场景各占一份的话，加三四个场景就吃光了。
 *
 * 规矩：
 *   · 只放"大块缓冲"。**字符串、步进、亮度这些参数不要放进来** ——
 *     它们很小，而且切走再切回来应该还在（放 union 里会被别的场景踩掉）。
 *   · 场景的 enter() 必须自己初始化自己那块，不能假设是干净的。
 *   · 流体不进这个 union —— 它 20 KB 常驻，塞进来会把 union 撑到 20 KB。
 */
#define APP_SCRATCH_ROW_BYTES   (32u)     /* 每行 32 字节 = 256 像素宽 */
#define APP_SCRATCH_ROWS        (16u)

typedef union {
    uint8_t text_px[APP_SCRATCH_ROWS][APP_SCRATCH_ROW_BYTES];  /* 1bpp 文字条带 */
    uint8_t snake_body[64];                                    /* 预留：贪吃蛇 */
    int8_t  ball[4];                                           /* 预留：平衡球 */
} app_scratch_t;

extern app_scratch_t s_scratch;

/* 进入主循环（含全部初始化），永不返回 */
void App_Run(void);

/* 每帧一次：读输入 → 全局事件 → 当前场景 tick */
void App_Tick(void);

/* 命令邮箱：全局命令自己处理，其余转给当前场景 */
void App_CmdTask(void);

/* 切到指定场景（越界按 0 处理）。已经是它就直接返回，不重入 enter/exit。 */
void App_Goto(uint8_t id);

/* 相对切换：+1 下一个 / -1 上一个（循环）*/
void App_Step(int8_t d);

/* 上位机字形条带：按字拼好后交给文字场景滚动（会切到场景 2）。
 * payload 格式见 TextScene_ComposeGlyphs()。返回 1 = 成功。 */
uint8_t App_ShowGlyphs(const uint8_t *p, uint16_t len, uint8_t count, int8_t spacing);

/* 把 Flash 里存的自定义滚动消息应用到文字场景（含滚动参数）。返回 1 = 成功。 */
uint8_t App_ApplyStoredGlyphs(void);

/* 上位机条带（整条长图）：切到文字场景并灌入条带，由 MCU 自己横向滚动。
 * bits = 16 行 × ceil(cols/8) 字节，行优先，MSB = 最左。返回 1 = 成功。 */
uint8_t App_ShowStrip(const uint8_t *bits, uint16_t cols);

/* 上位机直显：把一帧 240 B 像素（行优先 idx = row×15+col，0~255）显示出来，
 * 并切到"单帧画面"场景（编号 3）。
 * 串口二进制包（0xA5 开头）解析后调用这里；也可以直接调。 */
void App_ShowDirectFrame(const uint8_t *px);

/* 当前场景号 / 场景总数 */
uint8_t App_Current(void);
uint8_t App_Count(void);

/* 全局亮度（0x37）。低电安静窗口熄屏采样后要恢复成**用户设的这个值**，
 * 不能写死 MAX —— 否则串口把 bc 调到 64，每 2 秒就被打回 255。 */
void    App_SetBrightness(uint8_t bc);
uint8_t App_Brightness(void);

#endif /* __APP_H */
