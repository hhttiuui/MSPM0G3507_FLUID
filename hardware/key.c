/*
 * key.c —— 3 路按键的采样与去抖实现
 *
 * 分工：本文件只负责"引脚电平 → 稳定的按下/松开状态 + 边沿标志"，
 *       观测量写进 dbg.key；开关机逻辑在 power.c（它只调 Key_IsDown()）。
 */

#include "ti_msp_dl_config.h"
#include "debug.h"
#include "key.h"

/* 三根脚的表：顺序必须和 key_id_t 一致 */
typedef struct {
    GPIO_Regs *port;
    uint32_t   pin;
} key_pin_t;

static const key_pin_t s_keyPin[KEY_NUM] = {
    { GPIO_USERKEY_PORT, GPIO_USERKEY_PIN_USER1_PIN },
    { GPIO_USERKEY_PORT, GPIO_USERKEY_PIN_USER2_PIN },
    { GPIO_USERKEY_PORT, GPIO_USERKEY_PIN_USER3_PIN },
};

/* 去抖后的稳定状态：1 = 按下。初值在 Key_Init() 里用当时的电平填。
 * ⚠ s_stable / s_pressed 由 TIMA0 中断改、被主循环读，所以必须 volatile ——
 *   否则编译器（尤其开了 LTO 时）可能把主循环里的读提到循环外，读到永远不变的值。
 *   s_differ 只在中断里用，不需要 volatile。 */
static volatile uint8_t s_stable[KEY_NUM];
static uint8_t          s_differ[KEY_NUM];   /* 已连续多少拍读到"和稳定值不一样" */
static volatile uint8_t s_pressed[KEY_NUM];  /* "按下"边沿标志，Key_TookPress() 读后清 */
static volatile uint8_t s_released[KEY_NUM]; /* "松开"边沿标志，Key_TookRelease() 读后清 */
static uint8_t          s_armRel[KEY_NUM];   /* 1 = 见过一次"按下"，这次松开才算数 */

/* 原始电平 → 是否按下。三根脚都是"按下 = 低"（上拉 + 按键接地）；
 * 以后若有某根脚接法是"按下 = 高"，就在这里按 id 分开处理。 */
static uint8_t KeyRawIsDown(key_id_t k)
{
    return (DL_GPIO_readPins(s_keyPin[k].port, s_keyPin[k].pin) == 0u) ? 1u : 0u;
}

void Key_Init(void)
{
    uint32_t i;

    for (i = 0u; i < (uint32_t) KEY_NUM; i++) {
        /* ⚠ 反过来要出事：如果这里强行把初值写成"松开"，而开机按键正被按着
         * （一键开关机就是这样开机的），去抖器会在 20ms 后把它当成"刚按下"，
         * 于是一次开机动作被记成一次新的按键事件。所以初值必须取当前真实电平。 */
        s_stable[i]  = KeyRawIsDown((key_id_t) i);
        s_differ[i]  = 0u;
        s_pressed[i] = 0u;
        s_released[i] = 0u;

        /* ⚠ 松发边沿必须"上电后才武装"：
         *   一键开关机就是靠**按住 USER1** 上电的，如果开机时正按着的那次松开
         *   也被当成一次"松开事件"，一上电就会立刻切一次大类。
         *   所以初值一律 0：必须等到真的看见一次"按下"，之后的松开才算数。 */
        s_armRel[i]  = 0u;
    }
}

void Key_Task(void)
{
    uint32_t i;
    uint8_t  raw;
    uint8_t  rawBits = 0u;
    uint8_t  downBits = 0u;

    for (i = 0u; i < (uint32_t) KEY_NUM; i++) {
        raw = KeyRawIsDown((key_id_t) i);

        if (raw == s_stable[i]) {
            s_differ[i] = 0u;               /* 和稳定值一致 → 重新计数 */
        } else if (++s_differ[i] >= KEY_DEBOUNCE_TICKS) {
            s_stable[i] = raw;              /* 连续 4 拍（20ms）都变了 → 承认 */
            s_differ[i] = 0u;
            if (raw != 0u) {
                s_pressed[i] = 1u;          /* 记一次"按下"边沿 */
                s_armRel[i]  = 1u;          /* 这次按下之后，松开才有效 */
                dbg.key.press_cnt[i]++;
            } else if (s_armRel[i] != 0u) {
                s_released[i] = 1u;         /* 记一次"松开"边沿（必须是武装过的）*/
                s_armRel[i]   = 0u;
            }
        }

        if (raw != 0u) {
            rawBits |= (uint8_t) (1u << i);
        }
        if (s_stable[i] != 0u) {
            downBits |= (uint8_t) (1u << i);
        }
    }

    dbg.key.raw  = rawBits;                 /* bit0 = USER1 … bit2 = USER3 */
    dbg.key.down = downBits;
}

uint8_t Key_IsDown(key_id_t k)
{
    return s_stable[k];
}

uint8_t Key_TookPress(key_id_t k)
{
    uint8_t v = s_pressed[k];

    s_pressed[k] = 0u;                      /* 读后清 */
    return v;
}

uint8_t Key_TookRelease(key_id_t k)
{
    uint8_t v = s_released[k];

    s_released[k] = 0u;                     /* 读后清 */
    return v;
}
