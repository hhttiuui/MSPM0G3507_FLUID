/*
 * asset_route.h —— 两级路由：大类（cat）× 小类（item）→ 场景 + 资源
 *
 * ==================== 为什么要有这一层 ====================
 *
 *   s_scenes[] 是扁平表，**顺序即场景号**（串口 `mode <n>` 的编号），
 *   见 scenes.h 的警告。但用户心智模型是"大类 → 小类"：
 *
 *       流体（1 项） / 文字（N 组） / 图像动图（N 个槽） / 几何动画（M 种）
 *
 *   把按键直接绑到场景号，加一种动图就要改按键逻辑；所以中间加一层：
 *
 *       USER1 → 切大类      USER3 → 切小类
 *                     ↓
 *              (cat, item) → Route_Apply() → scene + 资源槽
 *
 *   RAM 代价：s_cat + s_item[4] ≈ 5 B。断电记忆走设置记录（flash_store）。
 *
 * ==================== 实现在 app.c ====================
 *
 *   本工程 `Debug` 下的 `subdir_vars.mk` 是 CCS 生成的显式文件列表，命令行 gmake
 *   不会重新扫描源码树，新增 .c 会在链接期才报 undefined。为了保持
 *   "一条 gmake 就能验证"的工作流，本模块只出声明，实现放在 app.c
 *   （它本来就拥有 App_Goto 与按键消费）。
 *
 *   见 多资源与两级按键方案.md §4/§5。
 */
#ifndef ASSET_ROUTE_H
#define ASSET_ROUTE_H

#include <stdint.h>

typedef enum {
    ROUTE_CAT_FLUID = 0,    /* 流体模拟      → scene 0 */
    ROUTE_CAT_TEXT,         /* 文字滚动      → scene 2 */
    ROUTE_CAT_IMAGE,        /* 静态图 / 动图 → scene 3 */
    ROUTE_CAT_GEOM,         /* 内置几何动画  → 阶段 4 才有场景 */
    ROUTE_CAT_COUNT
} route_cat_t;

/* 上电：从设置记录恢复 (cat, item)，并顺延掉空大类。只读 Flash，不绘制。 */
void Route_Init(void);

/* 上电：按恢复出来的 (cat, item) 进入对应场景。要在 Route_Init() 之后调。 */
void Route_Boot(void);

/* 主循环每圈调一次：状态脏了且空闲够久才落盘（避免每次按键都擦扇区）。 */
void Route_Tick(void);

/* USER1 短按：切到下一个非空大类，并恢复该大类上次的小类。 */
void Route_KeyCat(void);

/* USER3 短按：在当前大类内切小类；只有 1 项时无动作。 */
void Route_KeyItem(void);

/* 当前状态（调试/遥测用） */
uint8_t Route_Cat(void);
uint8_t Route_Item(void);

/* 指定大类有几项。阶段 2/3 会把 TEXT/IMAGE 换成读 Flash 里的真实条数。 */
uint8_t Route_ItemCount(uint8_t cat);

#endif /* ASSET_ROUTE_H */
