/**
 ****************************************************************************************************
 * @file        atk_key.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-21
 * @brief       key按键 驱动代码
 *
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20200421
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __ATK_KEY_H
#define __ATK_KEY_H


#include <board.h>
#include <os_task.h>

#define KEY0_PRES    1              /* KEY0按下 */
#define KEY1_PRES    2              /* KEY1按下 */
#define WKUP_PRES    3              /* KEY_UP按下(即WK_UP) */

uint8_t key_scan(uint8_t mode);

#endif
