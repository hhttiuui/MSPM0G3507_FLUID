/*
 * SysConfig Configuration Steps:
 *   I2C:
 *     1. Add an I2C module.
 *     2. Name it as "I2C_OLED".
 *     3. Check the box "Enable Controller Mode".
 *     4. Set "Standard Bus Speed" to "Fast Mode (400kHz)". (optional)
 *     5. Set the pins according to your needs.
 */
 
#ifndef __OLED_HARDWARE_I2C_H
#define __OLED_HARDWARE_I2C_H

#include "ti_msp_dl_config.h"

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

//OLED控制用函数
void delay_ms(uint32_t ms);
void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t sizey);
uint32_t oled_pow(uint8_t m,uint8_t n);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t sizey);
void OLED_ShowSignedNum(uint8_t x,uint8_t y,int32_t num,uint8_t len,uint8_t sizey);
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t sizey);
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t no,uint8_t sizey);
void OLED_DrawBMP(uint8_t x,uint8_t y,uint8_t sizex, uint8_t sizey,uint8_t BMP[]);
void OLED_Init(void);
void oled_i2c_sda_unlock(void);

/* ============================================================
 * 批量（突发）发送接口
 *
 * OLED_WR_Byte() 是"一个字节一次 I2C 事务"（约 150~250us/字节），
 * 全屏 1024 字节要 150~250ms，无法用于动画。
 * 下面三个接口把"控制字节 + n 字节负载"合并成**一次**事务：
 *   一页 61 字节 约 1.4ms，画布 8 页 约 13ms。
 *
 * 原有逐字节接口全部保留（菜单等旧代码继续使用，互不影响）。
 * 返回值：0 = 成功，1 = 失败/超时（内部会自动做 SDA 解锁恢复）。
 * ============================================================ */
#define OLED_I2C_ADDR   (0x3C)      /* SSD1306 从机地址 (SA0=0) */

/* 一次事务发送 n 条命令（自动加命令控制字节 0x00） */
int OLED_CmdSeq(uint8_t *cmds, uint16_t n);

/* 一次事务发送 n 字节显存数据（自动加数据控制字节 0x40） */
int OLED_DataBurst(uint8_t *data, uint16_t n);

/* 先设页地址/列地址，再把 buf 里 (x1-x0+1) 字节推到 [x0,x1] */
int OLED_PushPageRange(uint8_t page, uint8_t x0, uint8_t x1, uint8_t *buf);

#endif /* #ifndef __OLED_HARDWARE_I2C_H */
