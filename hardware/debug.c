/*
 * debug.c —— 观测支撑的全部实现：窗口统计（min/max）+ HardFault 现场捕获
 *
 * 全部观测量收在一个结构体里，除下面这几个"运行时实验开关"的初值之外一律零初始化。
 * 加新观测量只需要改 debug.h 的 dbg_t，不用动这个文件。
 *
 * 注意：驱动（lsm6ds3trc.c）只负责往 dbg 里写**瞬时值**，窗口统计（min/max）
 * 只在本文件里维护，避免"两个人的代码都在写同一个 min"这种排查不出来的鬼故事。
 */

#include "debug.h"

volatile dbg_t dbg = {
    /* ⚠ 窗口量的初值：min 必须显式给 0xFFFFFFFF。
     * 结构体默认零初始化会让 (d < min) 永远不成立，症状是 min 恒显示 0，
     * 看起来像"这个操作不耗时"。这里的 0xFFFFFFFF 语义是"还没发布过窗口"，
     * 配合 dbg.perf.win == 0 一起读。 */
    .perf = {
        .read_min_us = 0xFFFFFFFFu,
    },
    .test = {
        .throttle_ms = 1u,     /* 采样间隔 1ms ≈ 1kHz（传感器 ODR 只有 416Hz，
                                * 所以比它快就够了；0 = 不延时、跑满） */
        .publish_ms  = 400u,   /* 每 400ms 发布一次窗口统计（顺带翻心跳 LED） */
        .heartbeat   = 1u,     /* 心跳：主循环卡住时 LED 就不闪了，一眼能看出来 */
    },
};

/* ==================== 观测窗口：min/max 定期清除 ====================
 * 为什么必须做：全局单调的 min/max 一旦被"上电初期的一次性异常"污染就永远洗不掉
 * （比如第一次读取夹在复位等待里的那次超长耗时）。做法是"发布 + 清零"：
 * 累加器只在窗口内累计，窗口结束时整体搬到 dbg，然后清零重算。
 * 这样读到的**永远是上一个完整窗口**的结果，不会读到只填了几个样本、
 * min/max 都不可信的半成品窗口。
 *
 * 于是它同时成了判据：
 *   - 新窗口的 read_max_us 仍然异常大 → 异常**正在持续发生**
 *   - 回到十几 us                     → 那只是上电初期的一次性事件
 */
static uint32_t s_readMinUs = 0xFFFFFFFFu;   /* 当前窗口的工作累加器 */
static uint32_t s_readMaxUs = 0u;

void DBG_PerfAcc(uint32_t us)
{
    if (us < s_readMinUs) {
        s_readMinUs = us;
    }
    if (us > s_readMaxUs) {
        s_readMaxUs = us;
    }
}

void DBG_PerfPublish(void)
{
    dbg.perf.read_min_us = s_readMinUs;
    dbg.perf.read_max_us = s_readMaxUs;
    dbg.perf.win++;

    s_readMinUs = 0xFFFFFFFFu;
    s_readMaxUs = 0u;
}

/* ==================== HardFault 现场 ====================
 * 只记 PC / LR 两个值 —— 够定位到出错那条指令和调用来源了。
 * 不返回：出错后继续跑也没有意义，停在原地让 CCS 能读到 dbg.fault。
 *
 * 线程态和中断态都用 MSP，所以直接 mrs r0, msp 即可
 * （若以后用了 RTOS 的 PSP，这里要按 EXC_RETURN 的 bit2 选栈）。 */
void FaultCapture(uint32_t *sp)
{
    dbg.fault.cnt++;
    dbg.fault.pc = sp[6];   /* 出错那条指令的地址 */
    dbg.fault.lr = sp[5];   /* 从哪调过来的 */

    while (1) {
    }
}

/* ==================== 命令邮箱 ====================
 *
 * 串口（vcmd.c）和调试器都只负责"投递"，真正执行在主循环里
 * （app.c 的 App_CmdTask）。这样命令生效的时机是固定的，
 * 不会在 SPI 推屏中途改显示状态。
 *
 * ⚠ 顺序不能改：arg 先写、id 次之、pending 最后置位才算"提交完成"。
 *   中间任何一拍主循环看到 pending = 0 就什么都不做，是安全的。 */
void DBG_CmdSubmit(uint16_t id, int32_t a0, int32_t a1, int32_t a2, int32_t a3)
{
    dbg.cmd.arg[0]  = a0;
    dbg.cmd.arg[1]  = a1;
    dbg.cmd.arg[2]  = a2;
    dbg.cmd.arg[3]  = a3;
    dbg.cmd.id      = id;
    dbg.cmd.pending = 1u;

    dbg.uart.last_id = id;
}

__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile (
        "mrs r0, msp\n"     /* 本工程线程态和中断态都用 MSP */
        "b   FaultCapture\n"
    );
}
