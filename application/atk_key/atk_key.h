/**
 ****************************************************************************************************
 * @file        atk_key.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-21
 * @brief       key���� ��������
 *
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32F103������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20200421
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#ifndef __ATK_KEY_H
#define __ATK_KEY_H


#include <board.h>
#include <os_task.h>

#define KEY0_PRES    1              /* KEY0���� */
#define KEY1_PRES    2              /* KEY1���� */
#define WKUP_PRES    3              /* KEY_UP����(��WK_UP) */

uint8_t key_scan(uint8_t mode);

#endif
