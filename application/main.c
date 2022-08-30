/* 索引
include
define
int
信号量
TASK配置及函数声明
函数
主函数
*/

//MOLINK
#include <board.h>
#include <os_task.h>
#include <oneos_config.h>
#include <dlog.h>
#include <shell.h>
#include <fal/fal.h>
#include <os_memory.h>
#include <rtc/rtc.h>
#include <os_clock.h>
#include "atk_tflcd9341.h"
#include <mo_api.h>
#include <stdlib.h>
#include <unistd.h>
#include "MQTTOneOS.h"
#include "oneos_config.h"
#include "token.h"
#include "onenet_mqtts.h"
#include <esp8266.h>
//OTA
#include <cmiot_user.h>

//ATK
#include "atk_lcd.h"
#include "atk_key.h"

//RC522
#include "RC522.h"

//PWM
#include <drv_cfg.h>
#include <device.h>
#include <timer/clocksource.h>

//设备
#include "my_device.h"

//SHT40
//#include "sht40/sht4x.h"

//上传的消息
#include "upload_msg.h"

//传感器
#include <stdio.h>
#include <sensors/sensor.h>


#define RECV_BUF_LEN     (1024)
#define TEST_MODULE_NAME "esp8266"
#define AP_SSID          "201RIGHT"
#define AP_PASSWORD      "erlingyiyou"

//MQTT收到的控制数据（1字节）
char mqtt_receive_data;

//传感器sensor数据
//传感器name： temp_aht10 humi_aht10 
struct os_sensor_data sensor_data_temp;
struct os_sensor_data sensor_data_humi;


//设备定义
os_device_t *adc_1;
os_device_t *adc_2;
os_device_t *adc_3;
os_device_t *os_uart_3;
os_pwm_device_t *pwm_dev = OS_NULL;
os_timer_t *TIMER_PERIODIC = OS_NULL;
//os_device_t *sensor_temp;
//os_device_t *sensor_humi;
/********************************************************************************************************/
/* 数据定义 */
//当前设备状态
//struct device_state{
//	unsigned int  temperature;	//温度
//	unsigned int  humidity;		//湿度
//	unsigned int  power;		//功率
//	unsigned char  sound;		//噪声(数字量)
//	unsigned char light_d;		//光强（数字量）
//	unsigned int  light_b;		//板载光强
//	unsigned int  light;		//光强
//	unsigned char mq2_d;		//烟雾（数字量）
//	unsigned int  mq2;			//烟雾
//	unsigned char box1;			//器件盒1（数字量）
//	unsigned char box2;			//器件盒2（数字量）
//	unsigned char touch;		//触摸开关（数字量）
//	unsigned char human;		//人体存在（数字量）
//	unsigned char door;			//门（数字量）
//	
//	unsigned char motor;		//电机状态
//	unsigned char alarm;		//警报状态
//	unsigned char relay1;		//继电器1状态
//	unsigned char relay2;		//继电器2状态
//	
//}device_state;

//控制设备
struct device_control{

	unsigned char door;			//门
	//四合一
//	unsigned char motor;		//电机
//	unsigned char alarm;		//警报
//	unsigned char relay1;		//继电器1
//	unsigned char relay2;		//继电器2
	//四合一
	unsigned char d_switch; 	//控制其他设备

	
}device_control;

struct lcd_conf{

	int page;			//页
	int cursor;			//指针
	
	
}lcd_conf;



/********************************************************************************************************/
/* 信号量定义 */
static os_sem_t* sem_ota;	//ota信号量
static os_sem_t* sem_lcd;	//lcd信号量
//extern os_sem_t* cmd_device;	//cmd_device信号量
os_sem_t* sem_cmd_device;	//cmd_device信号量
os_sem_t* sem_lend_device;	//lend_device信号量
//os_sem_t* sem_auto_alarm;	//auto_alarm信号量
os_sem_t* sem_door;			//door信号量
os_sem_t* sem_nfc;			//nfc信号量
os_sem_t* sem_lend;			//外借任务信号量
os_sem_t* sem_lend_post;			//外借任务信号量
/* oneOS配置************************************************************************************** */
/* MQTT_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */

//MQTT任务配置
#define MQTT_TASK_PRIO    6              /* 任务优先级 */
#define MQTT_STK_SIZE     2048           /* 任务堆栈大小 */
os_task_t *MQTT_Handler;                 /* 任务控制块 */
void mqtt_task(void *parameter); 		  /* 任务函数 */


//KEY任务配置
#define KEY_TASK_PRIO    7              /* 任务优先级 */
#define KEY_STK_SIZE     512            /* 任务堆栈大小 */
os_task_t *KEY_Handler;                 /* 任务控制块 */
void key_task(void *parameter); 		 /* 任务函数 */


//OTA任务配置
#define OTA_TASK_PRIO    17             /* 任务优先级 */
#define OTA_STK_SIZE     2048           /* 任务堆栈大小 */
os_task_t *OTA_Handler;                 /* 任务控制块 */
void ota_task(void *parameter); 		 /* 任务函数 */

//LCD任务配置
#define LCD_TASK_PRIO    9              /* 任务优先级 */
#define LCD_STK_SIZE     1024           /* 任务堆栈大小 */
os_task_t *LCD_Handler;                 /* 任务控制块 */
void lcd_task(void *parameter);			 /* 任务函数 */

//CMD_DEVICE任务配置
#define CMD_DEVICE_TASK_PRIO    5              /* 任务优先级 */
#define CMD_DEVICE_STK_SIZE     512            /* 任务堆栈大小 */
os_task_t *CMD_DEVICE_Handler;                 /* 任务控制块 */
void cmd_device(void *parameter); 				  /* 任务函数 */

//LEND_DEVICE任务配置
#define LEND_DEVICE_TASK_PRIO    14            /* 任务优先级 */
#define LEND_DEVICE_STK_SIZE     512           /* 任务堆栈大小 */
os_task_t *LEND_DEVICE_Handler;                /* 任务控制块 */
void lend_device(void *parameter); 			  /* 任务函数 */
//AUTO_ALARM任务配置
#define AUTO_ALARM_TASK_PRIO    11             /* 任务优先级 */
#define AUTO_ALARM_STK_SIZE     512            /* 任务堆栈大小 */
os_task_t *AUTO_ALARM_Handler;                 /* 任务控制块 */
void auto_alarm(void *parameter); 				  /* 任务函数 */

//MQTT_CMD任务配置
#define MQTT_CMD_TASK_PRIO    10             /* 任务优先级 */
#define MQTT_CMD_STK_SIZE     512            /* 任务堆栈大小 */
os_task_t *MQTT_CMD_Handler;                 /* 任务控制块 */
void mqtt_cmd(void *parameter);				   /* 任务函数 */

//RC522任务配置
#define RC522_TASK_PRIO    18             /* 任务优先级 */
#define RC522_STK_SIZE     512            /* 任务堆栈大小 */
os_task_t *RC522_Handler;                 /* 任务控制块 */
void rc522_task(void *parameter);		   /* 任务函数 */

//DOOR任务配置
#define DOOR_TASK_PRIO    17             /* 任务优先级 */
#define DOOR_STK_SIZE     512            /* 任务堆栈大小 */
os_task_t *DOOR_Handler;                 /* 任务控制块 */
void door_task(void *parameter);		  /* 任务函数 */

//ADC任务配置
#define ADC_TASK_PRIO    11             /* 任务优先级 */
#define ADC_STK_SIZE     1024           /* 任务堆栈大小 */
os_task_t *ADC_Handler;                 /* 任务控制块 */
void adc_task(void *parameter); 		 /* 任务函数 */

//NFC任务配置
#define NFC_TASK_PRIO    11             /* 任务优先级 */
#define NFC_STK_SIZE     1024           /* 任务堆栈大小 */
os_task_t *NFC_Handler;                 /* 任务控制块 */

//LEND POST任务配置
#define LEND_TASK_PRIO    10             /* 任务优先级 */
#define LEND_STK_SIZE     512            /* 任务堆栈大小 */
os_task_t *LEND_Handler;                 /* 任务控制块 */
void lend_cmd(void *parameter); 		  /* 任务函数 */

// SERIAL_TASK 任务 配置
#define SERIAL_TASK_PRIO        18       /* 任务优先级 */
#define SERIAL_STK_SIZE         512      /* 任务堆栈大小 */
os_task_t *SERIAL_Handler;               /* 任务控制块 */
void serial_task(void *parameter);      /* 任务函数 */

#define USART2_MAX_RX_LEN    20                 /* 最大接收缓存字节数 */
os_uint8_t USART2_RX_BUF[USART2_MAX_RX_LEN];    /* 接收缓冲,最大USART2_MAX_RECV_LEN个字节. */


os_uint8_t k_back=0;		//按下返回键
os_uint8_t k_ok=0;		//按下确认键
char is_using_lend=0;	//lend模式
void nfc_task(void *parameter); //rc522检测
/********************************************************************************************************/
/* 构建消息 */
//其余的在upload_msg.h中
extern struct os_mq mqtts_mq;
/********************************************************************************************************/
/*子函数*/

/**
 * @brief       串口2设备初始化（用于从CI1122接收数据）
 * @param       无
 * @retval      返回设备句柄
 */
os_device_t *os_usart_2_init()
{
    os_device_t *os_uart_2;
    os_uart_2 = os_device_find("uart2");                            /* 寻找设备 */
    os_device_open(os_uart_2);                                      /* 打开设备 */
    struct serial_configure config = OS_SERIAL_CONFIG_DEFAULT;      /* 设置默认 */
    os_device_control(os_uart_2, OS_DEVICE_CTRL_CONFIG, &config);   /* 管理设备 */
    
    return os_uart_2;
}

/**
 * @brief       serial_task（从CI1122开发板接收数据）
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */
static void serial_task(void *parameter)
{
    parameter = parameter;
    os_uint32_t rx_cnt = 0;
    os_uint8_t i;
    memset(USART2_RX_BUF, 0, USART2_MAX_RX_LEN); 
    os_device_t *os_uart_2 = os_usart_2_init();
    for (i = 0; i < key_table_size; i++)
    {
        os_pin_mode(key_table[i].pin, key_table[i].mode);
    }
    char cmd_h;
    while (1)
    {
        rx_cnt = os_device_read_nonblock(os_uart_2,0,USART2_RX_BUF,USART2_MAX_RX_LEN);
        
        if (rx_cnt != 0 && rx_cnt > 0)
        {
			  		cmd_h=USART2_RX_BUF[0]>>4;
					switch (cmd_h)
					 {
						 case 6:case 2:case 3:case 4:{
							  os_kprintf("Recv submessage cmd: cmd_type=0x1-4x\r\n");
							  device_control.d_switch=USART2_RX_BUF[0];
							  os_sem_post(sem_cmd_device);
							  break;
						 }
						 case 5:{
							  os_kprintf("Recv submessage cmd: cmd_type=0x5x\r\n");	 
							  device_control.door=USART2_RX_BUF[0];
							  os_sem_post(sem_door);
							// auto_alarm_num=1;
							  break;
						 }
						 default:
									  break;
					  }
					 memset(USART2_RX_BUF, 0, USART2_MAX_RX_LEN);
				  }
		   os_task_msleep(100);
    }
}



/**
 * @brief       初始化（屏幕 molink）
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */
int all_init(){
   my_device_init();
	
	device_state.sound_num=0;
	device_state.light_num=0;
	device_state.mq2_num=0;
	
	lcd_conf.page=0;
	lcd_conf.cursor=0;
	
	cmiot_int8 rst_update = E_CMIOT_FAILURE;
	
	adc_1 = os_device_find("adc1");
	adc_2 = os_device_find("adc2");
	adc_3 = os_device_find("adc3");
//	OS_ASSERT_EX(OS_NULL != adc_1, "adc1 device not find! \r\n");
	OS_ASSERT_EX(OS_NULL != adc_2, "adc2 device not find! \r\n");
	OS_ASSERT_EX(OS_NULL != adc_3, "adc3 device not find! \r\n");
	os_device_open(adc_1);
	os_device_open(adc_2);
	os_device_open(adc_3);
//	adc_1 = os_device_open_s("adc1");
//	adc_2 = os_device_open_s("adc2");
//	adc_3 = os_device_open_s("adc3");
	os_device_control(adc_1, OS_ADC_CMD_ENABLE, OS_NULL);
	os_device_control(adc_2, OS_ADC_CMD_ENABLE, OS_NULL);
	os_device_control(adc_3, OS_ADC_CMD_ENABLE, OS_NULL);
    /* 手动创建模块 */
   mo_object_t *test_module = OS_NULL;
   mo_object_t *temp_module = OS_NULL;
   os_err_t ret = OS_EOK;
   lcd_show_string(30, 10, 200, 16, 16, "iZone V1.0", RED);
	lcd_show_string(30, 30, 200, 16, 16, "Welcome!!!", DARKBLUE);

	
	
	lcd_show_string(30, 70, 200, 16, 16, "Connnecting WiFi...", DARKBLUE);
   os_uart_3 = os_device_find("uart3");  /* 寻找设备 */
   struct serial_configure config = OS_SERIAL_CONFIG_DEFAULT;   /* 设置默认 */
   config.baud_rate = BAUD_RATE_115200;                         /* 设置波特率 */
   os_device_control(os_uart_3, OS_DEVICE_CTRL_CONFIG, &config);/* 管理设备 */
    
    /* hardware reset esp8266 */
   esp8266_hw_rst(ESP8266_RST_PIN_NUM);
   os_task_msleep(3000);

   mo_parser_config_t parser_config = {.parser_name   = TEST_MODULE_NAME,
                                       .parser_device = os_uart_3,
                                       .recv_buff_len = RECV_BUF_LEN};
    
   test_module = mo_create("esp8266", MODULE_TYPE_ESP8266, &parser_config); /* 创建molink */
   OS_ASSERT(OS_NULL != test_module);

   temp_module = mo_get_default();    /* 获取molink句柄 */
   OS_ASSERT(OS_NULL != temp_module);

    /* esp8266 connect ap */
   ret = mo_wifi_connect_ap(test_module, AP_SSID, AP_PASSWORD);
   OS_ASSERT(OS_EOK == ret);
    

   rst_update = cmiot_report_upgrade();
   if (E_CMIOT_SUCCESS == rst_update)
    {
		os_kprintf("\r\n UPDATE_SUCCESS \r\n");
		lcd_show_string(30, 170, 200, 16, 16, "OTA success", BLACK);
    }
	 
   lcd_show_string(30, 110, 200, 16, 16, "Success connect WiFi", RED);
	 	
	lcd_show_string(30, 130, 200, 16, 16, "Press any KEY to start", BLACK);
	  
	  //lcd_show_string(30, 190, 200, 16, 16, "OTA to V2.0", RED);
	
	return 0;
}
/**
 * @brief       按键检测
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */
static void key_task(void *parameter)
{
    parameter = parameter;
    os_uint8_t i;
    os_uint8_t key = 0;
  
    for (i = 0; i < key_table_size; i++)
    {
        os_pin_mode(key_table[i].pin, key_table[i].mode);
    }
    
    while (1)
    {
        key = key_scan(0);
        
        if (key == WKUP_PRES)   		//返回
		  {
			  k_ok=0;
			  is_using_lend=0;
			  k_back=1;
           if(lcd_conf.page>0){
					lcd_conf.page=0;
					lcd_conf.cursor=0;
					os_sem_post(sem_lcd);
				}
        }
        else if(key == KEY1_PRES)	//确认
		  {
			  if(lcd_conf.page==3) k_ok=1; 
			  else{
				lcd_conf.page=lcd_conf.cursor;
				lcd_conf.cursor=0;
			   os_sem_post(sem_lcd);
			  }  
		  }
		  else if(key == KEY0_PRES)	//指针加一
		  {
			  lcd_conf.cursor++;
			  if(lcd_conf.cursor>4)lcd_conf.cursor=1;
			  os_sem_post(sem_lcd);		  
		  }	  
        os_task_msleep(10);		
    }

}

/**
 * @brief       LCD刷新
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */

static void lcd_task(void *parameter)
{
    parameter = parameter;
	 while(1){
		 if (OS_EOK == os_sem_wait(sem_lcd, OS_WAIT_FOREVER)){
			  switch(lcd_conf.page){
				  case(0):{
					   lcd_clear(WHITE);
					   lcd_show_string(30, 10, 200, 16, 16, "iZone", RED);
						lcd_show_string(30, 30, 200, 16, 16, "Device State", DARKBLUE);  //pag1
						lcd_show_string(30, 50, 200, 16, 16, "Device Control", DARKBLUE);//pag2
					   lcd_show_string(30, 70, 200, 16, 16, "Device Lend", DARKBLUE);   //pag3
					   lcd_show_string(30, 90, 200, 16, 16, "Device OTA", DARKBLUE);    //pag4
					   
					   break;
				  }					  
				  case(1):{
						lcd_clear(WHITE);
					   lcd_show_string(30, 10, 200, 16, 16, "Device State", BLACK);
					  break;
				  }
				  case(2):{
						lcd_clear(WHITE);
					   lcd_show_string(30, 10, 200, 16, 16, "Device Control", BLACK);
					   break;
				  }
				  case(3):{
						lcd_clear(WHITE);
					   lcd_show_string(30, 10, 200, 16, 16, "Device Lend", BLACK);
					   os_sem_post(sem_lend);//更改OTA信号量
					   break;
				  }
												  
				  case(4):{
				   	lcd_clear(WHITE);
					   lcd_show_string(30, 10, 200, 16, 16, "OTA", BLACK);
					   os_sem_post(sem_ota);//更改OTA信号量
					   break;
				  }
				  
			  }
			  if(lcd_conf.page==0&&lcd_conf.cursor>0)lcd_fill_circle(15, lcd_conf.cursor*20+18,5,LIGHTGREEN);
		 }
	 }
    
}

/**
 * @brief       OTA升级
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */

static void ota_task(void *parameter)
{
    parameter = parameter;
    cmiot_int8 rst = E_CMIOT_FAILURE;
	 while(1){
		 if (OS_EOK == os_sem_wait(sem_ota, OS_WAIT_FOREVER)){
			  rst = cmiot_upgrade();
		//	 os_kprintf("\r\nrst: %d \r\n",&rst);
			  if (E_CMIOT_SUCCESS == rst){
				   lcd_show_string(30, 50, 200, 16, 16, "Download Success!", RED);
				   lcd_show_string(30, 70, 200, 16, 16, "Please Reset (Press RST) ", BLACK);
					os_kprintf("\r\n DOWNLOAD_SUCCESS \r\n");
					os_kprintf("\r\n PLEASE RESET \r\n");
	  //          lcd_show_string(30, 70, 200, 16, 16, "CMIOT_SUCCESS", BLUE);
			  }
			  else if (E_CMIOT_LAST_VERSION == rst){		  
					os_kprintf("\r\n LAST_VERSION \r\n");
				   lcd_show_string(30, 50, 200, 16, 16, "Last Version", RED);
			  }
		 }
	 }
    
}
/**
 * @brief       ONENET MQTT发送（publish消息太长 分成了6段）
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */
static void mqtt_task(void *parameter)
{
	
	    /* 连接onenet mqtt */
    onenet_mqtts_device_start();
	


	//MQTT接收后执行onenet_mqtts.c的submessage_cmd_request_arrived_handler
	 onenet_mqtts_device_subscribe();
	
    parameter = parameter;
    os_err_t rc;
    char pub_buf1[PUB_DATA_BUFF_LEN] = {0};
	 char pub_buf2[PUB_DATA_BUFF_LEN] = {0};
	 char pub_buf3[PUB_DATA_BUFF_LEN] = {0};
	 char pub_buf4[PUB_DATA_BUFF_LEN] = {0};
	 char pub_buf5[PUB_DATA_BUFF_LEN] = {0};
	 char pub_buf6[PUB_DATA_BUFF_LEN] = {0};
    char *pub_msg = OS_NULL;
    int pub_msg_len = 0;
    mq_msg_t mq_msg;
    int id = 0;

    int send_num = 0;

    while (1)
    {
        if (onenet_mqtts_device_is_connected() == 1) /* 判断设备是否连接 */
        {
            send_num ++;

            if (id != 2147483647)
            {
                id++;
            }
            else
            {
                id = 1;
            }
			/***********************************************/
			FILL_MSG1;//在upload_msg.h里
            pub_msg = pub_buf1;
            pub_msg_len = strlen(pub_msg);
            memset(&mq_msg, 0x00, sizeof(mq_msg));		//清零
            mq_msg.topic_type = DATA_POINT_TOPIC;		//主题类型
            memcpy(mq_msg.data_buf, pub_msg, pub_msg_len);	//将 pub_msg的值拷贝到消息队列的消息内容数组
            mq_msg.data_len = pub_msg_len;				//消息长度
//			os_kprintf("msg1_len %d \n", mq_msg.data_len);
            rc = os_mq_send(&mqtts_mq, (void *)&mq_msg, sizeof(mq_msg_t), 0);  /* 发送数据 */
            if (rc != OS_EOK)
            {
                os_kprintf("\r\n 1 mqtts_device_messagequeue_send ERR\r\n");
            }
				os_task_msleep(950);
			/***********************************************/
				id++;
				FILL_MSG2;//在upload_msg.h里
            pub_msg = pub_buf2;
            pub_msg_len = strlen(pub_msg);
            memset(&mq_msg, 0x00, sizeof(mq_msg));		//清零
            mq_msg.topic_type = DATA_POINT_TOPIC;		//主题类型
            memcpy(mq_msg.data_buf, pub_msg, pub_msg_len);	//将 pub_msg的值拷贝到消息队列的消息内容数组
            mq_msg.data_len = pub_msg_len;				//消息长度
//				 os_kprintf("msg2_len %d \n", mq_msg.data_len);
            rc = os_mq_send(&mqtts_mq, (void *)&mq_msg, sizeof(mq_msg_t), 0);  /* 发送数据 */
            if (rc != OS_EOK)
            {
                os_kprintf("\r\n 2 mqtts_device_messagequeue_send ERR\r\n");
            }
				os_task_msleep(950);
			/***********************************************/
				id++;
				FILL_MSG3;//在upload_msg.h里
            pub_msg = pub_buf3;
            pub_msg_len = strlen(pub_msg);
            memset(&mq_msg, 0x00, sizeof(mq_msg));		//清零
            mq_msg.topic_type = DATA_POINT_TOPIC;		//主题类型
            memcpy(mq_msg.data_buf, pub_msg, pub_msg_len);	//将 pub_msg的值拷贝到消息队列的消息内容数组
            mq_msg.data_len = pub_msg_len;				//消息长度
//				 os_kprintf("msg3_len %d \n", mq_msg.data_len);
            rc = os_mq_send(&mqtts_mq, (void *)&mq_msg, sizeof(mq_msg_t), 0);  /* 发送数据 */
            if (rc != OS_EOK)
            {
                os_kprintf("\r\n 3 mqtts_device_messagequeue_send ERR\r\n");
            }
				os_task_msleep(950);
			/***********************************************/
				id++;
				FILL_MSG4;//在upload_msg.h里
            pub_msg = pub_buf4;
            pub_msg_len = strlen(pub_msg);
            memset(&mq_msg, 0x00, sizeof(mq_msg));		//清零
            mq_msg.topic_type = DATA_POINT_TOPIC;		//主题类型
            memcpy(mq_msg.data_buf, pub_msg, pub_msg_len);	//将 pub_msg的值拷贝到消息队列的消息内容数组
            mq_msg.data_len = pub_msg_len;				//消息长度
//				 os_kprintf("msg4_len %d \n", mq_msg.data_len);
            rc = os_mq_send(&mqtts_mq, (void *)&mq_msg, sizeof(mq_msg_t), 0);  /* 发送数据 */
            if (rc != OS_EOK)
            {
                os_kprintf("\r\n 4 mqtts_device_messagequeue_send ERR\r\n");
            }
				os_task_msleep(950);
			/***********************************************/
				id++;
				FILL_MSG5;//在upload_msg.h里
            pub_msg = pub_buf5;
            pub_msg_len = strlen(pub_msg);
            memset(&mq_msg, 0x00, sizeof(mq_msg));		//清零
            mq_msg.topic_type = DATA_POINT_TOPIC;		//主题类型
            memcpy(mq_msg.data_buf, pub_msg, pub_msg_len);	//将 pub_msg的值拷贝到消息队列的消息内容数组
            mq_msg.data_len = pub_msg_len;				//消息长度
//				 os_kprintf("msg5_len %d \n", mq_msg.data_len);
            rc = os_mq_send(&mqtts_mq, (void *)&mq_msg, sizeof(mq_msg_t), 0);  /* 发送数据 */
            if (rc != OS_EOK)
            {
                os_kprintf("\r\n 5 mqtts_device_messagequeue_send ERR\r\n");
            }
				os_task_msleep(950);
        
		  
			/***********************************************/
				if(device_state.light_num>0||device_state.sound_num>0||device_state.mq2_num>0){
					id++;
					FILL_MSG6;//在upload_msg.h里
					pub_msg = pub_buf6;
					pub_msg_len = strlen(pub_msg);
					memset(&mq_msg, 0x00, sizeof(mq_msg));		//清零
					mq_msg.topic_type = DATA_POINT_TOPIC;		//主题类型
					memcpy(mq_msg.data_buf, pub_msg, pub_msg_len);	//将 pub_msg的值拷贝到消息队列的消息内容数组
					mq_msg.data_len = pub_msg_len;				//消息长度
	//				 os_kprintf("msg5_len %d \n", mq_msg.data_len);
					rc = os_mq_send(&mqtts_mq, (void *)&mq_msg, sizeof(mq_msg_t), 0);  /* 发送数据 */
					if (rc != OS_EOK)
					{
						 os_kprintf("\r\n 5 mqtts_device_messagequeue_send ERR\r\n");
					}
					os_task_msleep(950);
				}
        }
        os_task_msleep(1000);
    }
}
/**
 * @brief       MQTT接收到0x01后执行task1
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */
static void cmd_device(void *parameter)
{
	parameter=parameter;
	while(1){
		 if (OS_EOK == os_sem_wait(sem_cmd_device, OS_WAIT_FOREVER)){
			 os_kprintf("\r\n CMD_DEVICE \r\n");
			 my_device_cmd_control(device_control.d_switch);
			 os_kprintf("\r\n d_switch %d \r\n",device_control.d_switch);
			// device_control.d_switch=0;
		 } 
	 }
	
}
/**
 * @brief       外借任务
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */
static void lend_device(void *parameter)
{
	parameter=parameter;
	while(1){
		 if (OS_EOK == os_sem_wait(sem_lend, OS_WAIT_FOREVER)){
			 lcd_clear(WHITE);
			 is_using_lend=1;
			  for(int j=0;j<9;j++){
				device_state.nfc_id_last[j]=0;
				device_state.nfc_id_now[j]=0;
			  }				  
			 while(1){
				
//			 device_state.nfc_id_last[0]='a';
//			 device_state.nfc_id_last[1]='a';
//			 device_state.nfc_id_last[2]='a';
//			 device_state.nfc_id_last[3]='a';
//			 device_state.nfc_id_last[4]='a';
//			 device_state.nfc_id_last[5]='a';
//			 device_state.nfc_id_last[6]='a';
//			 device_state.nfc_id_last[7]='a';
//			 device_state.nfc_id_last[8]=0;
			 
			 os_kprintf("\r\n LEND_DEVICE \r\n");
			 lcd_show_string(30, 10, 200, 16, 16, "Device Lend", BLACK);
			 lcd_show_string(30, 30, 200, 16, 16, "Please swipe Card", DARKBLUE);  //pag1
			 lcd_show_string(30, 50, 200, 16, 16, device_state.nfc_id_last, RED);  //pag1
			 lcd_show_string(30, 70, 200, 16, 16, "Device RFID", DARKBLUE);  //pag1
			 lcd_show_string(30, 90, 200, 16, 16, device_state.nfc_id_now, RED);  //pag1
			 if(k_ok){
				 lcd_show_string(30, 130, 200, 16, 16, "Lend Over", RED);  //pag1
				 lcd_show_string(30, 150, 200, 16, 16, "Press UP to Exit", RED);  //pag1
				 if(device_state.nfc_id_last[0]!=0&&device_state.nfc_id_now[0]!=0)os_sem_post(sem_lend_post );		
				 k_ok=0;
				 break;
			 }
			 if(k_back) break;
			 os_task_msleep(1000);
			}
			 
		 } 
		 is_using_lend=0;
	 }
	
}
/**
 * @brief       设备联动(自动控制和上传警报)
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */
static void auto_alarm(void *parameter)
{
	parameter=parameter;
	while(1){

		 if(device_state.mq2_d==1) {
			device_state.mq2_num++;
			device_control.d_switch=0x61;
			os_sem_post(sem_cmd_device );		 
			device_control.d_switch=0x21;
			os_sem_post(sem_cmd_device );

			os_task_msleep(3000);
			device_control.d_switch=0x60;
			os_sem_post(sem_cmd_device );
			device_control.d_switch=0x20;
			os_sem_post(sem_cmd_device );

		 }
		 if(device_state.sound==1){
			device_state.sound_num++;
			device_control.d_switch=0x21;
			os_sem_post(sem_cmd_device );
			os_task_msleep(3000);
			device_control.d_switch=0x20;
			os_sem_post(sem_cmd_device );
		 }
		 if(device_state.light_d==1){
			 device_state.light_num++;
		 }
		 
//		 	os_kprintf("\r\nsound=%d\r\n",device_state.sound_num);	 
//		  os_kprintf("\r\nMQ2=%d\r\n",device_state.mq2_num);	
//		 os_kprintf("\r\nlight=%d\r\n",device_state.light_num);	
		 os_task_msleep(500);
	 }
	
}

/**
 * @brief       外借上传
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */
static void lend_task(void *parameter)
{
	parameter=parameter;
	os_err_t rc;
   char pub_buf_lend[PUB_DATA_BUFF_LEN] = {0};

   char *pub_msg = OS_NULL;
   int pub_msg_len = 0;
   mq_msg_t mq_msg;
	while(1){
		 if (OS_EOK == os_sem_wait(sem_lend_post, OS_WAIT_FOREVER)){
				os_kprintf("\r\n MQTT_LEND \r\n");
				lcd_show_string(30, 190, 200, 16, 16, "LEND OK.", DARKBLUE);  //pag1
			  
			 
				FILL_MSG_LEND;//在upload_msg.h里
            pub_msg = pub_buf_lend;
            pub_msg_len = strlen(pub_msg);
            memset(&mq_msg, 0x00, sizeof(mq_msg));		//清零
            mq_msg.topic_type = DATA_POINT_TOPIC;		//主题类型
            memcpy(mq_msg.data_buf, pub_msg, pub_msg_len);	//将 pub_msg的值拷贝到消息队列的消息内容数组
            mq_msg.data_len = pub_msg_len;				//消息长度
				os_kprintf("msg_nfc: %s \n", mq_msg.data_buf);
            rc = os_mq_send(&mqtts_mq, (void *)&mq_msg, sizeof(mq_msg_t), 0);  /* 发送数据 */
            if (rc != OS_EOK)
            {
                os_kprintf("\r\n NFC mqtts_device_messagequeue_send ERR\r\n");
            }
				os_task_msleep(950);
		 } 
		
	 }
	
}


/**
 * @brief       nfc读取到ID后，publish ID与开关门操作
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */
static void nfc_task(void *parameter)
{
	parameter=parameter;
	os_err_t rc;
   char pub_buf_nfc[PUB_DATA_BUFF_LEN] = {0};

   char *pub_msg = OS_NULL;
   int pub_msg_len = 0;
   mq_msg_t mq_msg;
	while(1){
		 if (OS_EOK == os_sem_wait(sem_nfc, OS_WAIT_FOREVER)){
			 
				os_kprintf("\r \n nfc open door \r \n");
				device_control.door=0x52;
				os_sem_post(sem_door);
			 
				FILL_MSG_NFC;//在upload_msg.h里
            pub_msg = pub_buf_nfc;
            pub_msg_len = strlen(pub_msg);
            memset(&mq_msg, 0x00, sizeof(mq_msg));		//清零
            mq_msg.topic_type = DATA_POINT_TOPIC;		//主题类型
            memcpy(mq_msg.data_buf, pub_msg, pub_msg_len);	//将 pub_msg的值拷贝到消息队列的消息内容数组
            mq_msg.data_len = pub_msg_len;				//消息长度
				os_kprintf("msg_nfc_nfc_task: %s \n", mq_msg.data_buf);
            rc = os_mq_send(&mqtts_mq, (void *)&mq_msg, sizeof(mq_msg_t), 0);  /* 发送数据 */
            if (rc != OS_EOK)
            {
                os_kprintf("\r\n NFC mqtts_device_messagequeue_send ERR\r\n");
            }
				os_task_msleep(950);
			 
		 } 
	 }
	
}
/**
 * @brief       模拟量检测与传感器
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */
static void adc_task(void *parameter)
{
	parameter=parameter;
	os_device_t *sensor_temp = os_device_open_s("temp_aht10");
	os_device_t *sensor_humi = os_device_open_s("humi_aht10");
	
	struct os_sensor_info sensor_info_temp;
	struct os_sensor_info sensor_info_humi;

   os_device_control(sensor_temp, OS_SENSOR_CTRL_GET_INFO, &sensor_info_temp);
	os_device_control(sensor_humi, OS_SENSOR_CTRL_GET_INFO, &sensor_info_humi);


	int light_b;
	int power1=4;
	int power2=4;
	
	while(1){
		
		//sensor 温度
	   os_device_read_nonblock(sensor_temp, 0, &sensor_data_temp, sizeof(struct os_sensor_data));
		if (sensor_info_temp.unit == OS_SENSOR_UNIT_MDCELSIUS)
        {
//            os_kprintf("sensor temp (%d.%03d)\r\n", sensor_data_temp.data.temp / 1000, sensor_data_temp.data.temp % 1000);
				device_state.temperature=sensor_data_temp.data.temp / 1000;
		  }
        else if (sensor_info_temp.unit == OS_SENSOR_UNIT_DCELSIUS)
        {
//            os_kprintf("sensor temp (%d)\r\n", sensor_data_temp.data.temp);
			   device_state.temperature=sensor_data_temp.data.temp;
        }
        else
        {
            os_kprintf("invalid unit\r\n");
        }
		  //sensor 湿度
		os_device_read_nonblock(sensor_humi, 0, &sensor_data_humi, sizeof(struct os_sensor_data));
		if (sensor_info_humi.unit == OS_SENSOR_UNIT_MPERMILLAGE)
        {
//            os_kprintf("sensor humi (%d.%03d)\r\n", sensor_data_humi.data.humi / 1000, sensor_data_humi.data.humi % 1000);
				device_state.humidity=sensor_data_humi.data.humi / 1000;
		  }
        else if (sensor_info_humi.unit == OS_SENSOR_UNIT_PERMILLAGE)
        {
//            os_kprintf("sensor humi (%d)\r\n", sensor_data_humi.data.humi);
			   device_state.humidity=sensor_data_humi.data.humi;
        }
        else
        {
            os_kprintf("invalid unit\r\n");
        }
		  
	
		//功率检测
		// os_device_read_nonblock( adc_1, ADC_CHANNEL_12,&device_state.light,sizeof(device_state.light));//adc1无法使用
		os_device_read_nonblock( adc_2, 12,&power1,sizeof(power1));//POWER_SENSOR1_A_PIN
		os_device_read_nonblock( adc_2, ADC_CHANNEL_15,&power2,sizeof(power2));//POWER_SENSOR1_B_PIN
		 device_state.power=16*power1*(power1-power2)/30/1000;
//		  os_kprintf("\r\n power---->%d,%d,%d\r\n",power1,power2,device_state.power);
//		  //MQ2无效
//		 os_device_read_nonblock( adc_2, ADC_CHANNEL_10,&device_state.mq2,sizeof(device_state.mq2));
//		   os_kprintf("\r\n ch10---->%d\r\n",device_state.mq2);
		//板载光照（已验证）
		os_device_read_nonblock( adc_3, ADC_CHANNEL_6,&light_b,sizeof(light_b));
		device_state.light_b=(int)(0.06728*(3300-light_b));	 
		  
//		os_kprintf("------->%d\r\n",device_state.light_b);
		  
		os_task_msleep(1000);
	 }
	
}
/**
 * @brief       门开关
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */
static void door_task(void *parameter)
{
	
	
	parameter=parameter;
	
	os_uint32_t period = 20000000;  
   os_uint32_t close_pulse  = 1400000; //1500000  0度 1850000 63度     
	os_uint32_t open_pulse  = 1900000;
   os_uint32_t channel= 3;//PB8
	//板载蜂鸣器+门
	pwm_dev =  (os_pwm_device_t *)os_device_find("pwm_tim4");
	    if (pwm_dev == OS_NULL)
    {
        os_kprintf("pwm sample run failed! can't find device!\n");
       // return OS_ERROR;
    }
	//00 关闭
	//01 开启
	//02 先开后关
	os_pwm_set_period(pwm_dev, channel, period);

	while(1){
		 if (OS_EOK == os_sem_wait(sem_door, OS_WAIT_FOREVER)){
			switch(device_control.door){
				case 0x50: {
					device_state.door=0;
					os_pwm_set_pulse(pwm_dev, channel, close_pulse);
					os_pwm_enable(pwm_dev, channel);
					os_task_msleep(500);
	 				os_pwm_disable(pwm_dev, channel);
					break;
				}
				case 0x51: {
					device_state.door=1;
					os_pwm_set_pulse(pwm_dev, channel, open_pulse);
					os_pwm_enable(pwm_dev, channel);
					os_task_msleep(500);
	 				os_pwm_disable(pwm_dev, channel);
					break;
				}
				case 0x52: {
					device_state.door=1;
					os_pwm_set_pulse(pwm_dev, channel, open_pulse);
					os_pwm_enable(pwm_dev, channel);
					os_task_msleep(500);
	 				os_pwm_disable(pwm_dev, channel);
					
					os_task_msleep(3000);
					device_state.door=0;
					os_pwm_set_pulse(pwm_dev, channel, close_pulse);
					os_pwm_enable(pwm_dev, channel);
					os_task_msleep(500);
	 				os_pwm_disable(pwm_dev, channel);
					break;
				}
				
			}
			device_control.door=0;
		 } 
	 }
	
}
/*
 * @brief       TOUCH中断函数
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */
static void touch_irq(void *args)
{
//	os_task_msleep(20);
	if(os_pin_read(TOUCH_SENSOR_PIN)){
		device_control.door=0x52;
		os_kprintf("\r\n irq \r\n");
		os_sem_post(sem_door);
	
	}
}

/**
 * @brief       MQTT接收的数据检测
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */
static void mqtt_cmd(void *parameter)
{
	parameter=parameter;
	char cmd_h;
	while(1){
		cmd_h=mqtt_receive_data>>4;
		switch (cmd_h)
		 {
		 /* do own thing here */
		 case 6:case 2:case 3:case 4:{
			  os_kprintf("Recv submessage cmd: cmd_type=0x1-4x\r\n");
			  device_control.d_switch=mqtt_receive_data;
			  os_sem_post(sem_cmd_device);
			// cmd_device_num=1;
			  break;
		 }
		case 5:{
			  os_kprintf("Recv submessage cmd: cmd_type=0x5x\r\n");	 
			  device_control.door=mqtt_receive_data;
			  os_sem_post(sem_door);
			// auto_alarm_num=1;
			  break;
		}
		 default:
			  break;
		 }
		 mqtt_receive_data=0;
		 os_task_msleep(100);		
	 }
	
}
/**
 * @brief       RC522_TASK子函数、显示NFC的ID
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */
static void ShowID(unsigned char *p)
{

    unsigned char i;
	 for(i=0; i<9; i++)
    {		 
		 device_state.nfc_id_last[i]=device_state.nfc_id_now[i];
	 }
    for(i=0; i<4; i++)
    {
        device_state.nfc_id_now[i*2]=p[i]/16;
        device_state.nfc_id_now[i*2]>9?(device_state.nfc_id_now[i*2]+='7'):(device_state.nfc_id_now[i*2]+='0');
        device_state.nfc_id_now[i*2+1]=p[i]%16;
        device_state.nfc_id_now[i*2+1]>9?(device_state.nfc_id_now[i*2+1]+='7'):(device_state.nfc_id_now[i*2+1]+='0');
    }
    device_state.nfc_id_now[8]=0;
	 os_kprintf("last ID>>>%s\r\n", device_state.nfc_id_last);
    os_kprintf("ID>>>%s\r\n", device_state.nfc_id_now);

}

//static os_uint8_t rfid_stack[ 512 ];
//static struct rt_thread rfid_thread;
/**
 * @brief       RC522读取NFC的ID
 * @param       parameter : 传入参数(未用到)
 * @retval      无
 */
static void rc522_task(void* parameter)
{
//    unsigned int count=0;

   char status;
	//unsigned char snr, buf[16], TagType[2], SelectedSnr[4], DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
   unsigned char  TagType[2], SelectedSnr[4];
   PcdInit();
	PcdReset();
	PcdAntennaOff();
	PcdAntennaOn();
	M500PcdConfigISOType( 'A' );
 
   os_kprintf( "rc522 init over!\n" );

  while(1){
    status= PcdRequest( REQ_ALL , TagType );
		if(!status) {
			status = PcdAnticoll(SelectedSnr);
			if(!status){
				status=PcdSelect(SelectedSnr);
				if(!status){
					ShowID(SelectedSnr);//读取ID
				//	os_kprintf( "+++++++++++++++++++++++ %d\n" ,is_using_lend);
					if(!is_using_lend) {
				//	os_kprintf( "************************\n" );
					os_sem_post(sem_nfc);}
					WaitCardOff();
					 
				}
			}
		}
		os_task_msleep(500);
	}
}





int main(void)
{

	//初始化
    all_init();
	
  /* USER CODE BEGIN 2 */
	
	//信号量创建
	 sem_ota = os_sem_create("sem_ota", 0,1);
	 sem_lcd = os_sem_create("sem_lcd", 0,1);
	 sem_cmd_device= os_sem_create("sem_cmd_device", 0,1);//cmd_device信号量
    //sem_lend_device= os_sem_create("sem_lend_device", 0,1);//lend_device信号量
    //sem_auto_alarm= os_sem_create("sem_auto_alarm", 0,1);//auto_alarm信号量
	 sem_lend= os_sem_create("sem_lend", 0,1);//外借任务
	 sem_lend_post= os_sem_create("sem_lend_post", 0,1);//上传任务
	 sem_door = os_sem_create("sem_door", 0,1);//door
	 sem_nfc = os_sem_create("sem_nfc", 0,1);//nfc
	// OS_ASSERT_EX(OS_NULL != sem_ota, "sem_ota create err!\r\n");	
	
    /* 绑定中断，上升沿模式，回调函数名为touch_irq */
    os_pin_attach_irq(TOUCH_SENSOR_PIN, PIN_IRQ_MODE_RISING, touch_irq, OS_NULL);
    /* 使能中断 */
    os_pin_irq_enable(TOUCH_SENSOR_PIN, PIN_IRQ_ENABLE);
	 
	//任务创建
    MQTT_Handler = os_task_create(  "mqtt_task",   /* 设置任务的名称 */
                                  mqtt_task,     /* 设置任务函数 */
                                  OS_NULL,              /* 任务传入的参数 */
                                  MQTT_STK_SIZE,        /* 设置任务堆栈 */
                                  MQTT_TASK_PRIO);      /* 设置任务的优先级 */
    OS_ASSERT(MQTT_Handler);
    os_task_startup(MQTT_Handler);
	
	 KEY_Handler = os_task_create("key_task", key_task, OS_NULL, KEY_STK_SIZE, KEY_TASK_PRIO);
    OS_ASSERT(KEY_Handler);
    os_task_startup(KEY_Handler);
	
	 OTA_Handler = os_task_create("ota_task", ota_task, OS_NULL, OTA_STK_SIZE, OTA_TASK_PRIO);      
    OS_ASSERT(OTA_Handler);
    os_task_startup(OTA_Handler);
	 
	 LCD_Handler = os_task_create("lcd_task", lcd_task, OS_NULL, LCD_STK_SIZE, LCD_TASK_PRIO);      
    OS_ASSERT(LCD_Handler);
    os_task_startup(LCD_Handler);
	 
	 CMD_DEVICE_Handler = os_task_create("cmd_device", cmd_device, OS_NULL, CMD_DEVICE_STK_SIZE, CMD_DEVICE_TASK_PRIO);      
    OS_ASSERT(CMD_DEVICE_Handler);
    os_task_startup(CMD_DEVICE_Handler);
	 
	 LEND_DEVICE_Handler = os_task_create("lend_device", lend_device, OS_NULL, LEND_DEVICE_STK_SIZE, LEND_DEVICE_TASK_PRIO);      
    OS_ASSERT(LEND_DEVICE_Handler);
    os_task_startup(LEND_DEVICE_Handler);
	 
	 AUTO_ALARM_Handler = os_task_create("auto_alarm", auto_alarm, OS_NULL, AUTO_ALARM_STK_SIZE, AUTO_ALARM_TASK_PRIO);      
    OS_ASSERT(AUTO_ALARM_Handler);
    os_task_startup(AUTO_ALARM_Handler);
	 
	 MQTT_CMD_Handler = os_task_create("mqtt_cmd", mqtt_cmd, OS_NULL, MQTT_CMD_STK_SIZE, MQTT_CMD_TASK_PRIO);      
    OS_ASSERT(MQTT_CMD_Handler);
    os_task_startup(MQTT_CMD_Handler);
	 
	 RC522_Handler = os_task_create("rc522_task", rc522_task, OS_NULL, RC522_STK_SIZE, RC522_TASK_PRIO);      
    OS_ASSERT(RC522_Handler);
    os_task_startup(RC522_Handler);
	
	 DOOR_Handler = os_task_create("door_task", door_task, OS_NULL, DOOR_STK_SIZE, DOOR_TASK_PRIO);      
    OS_ASSERT(DOOR_Handler);
    os_task_startup(DOOR_Handler);
	 
	 ADC_Handler = os_task_create("adc_task", adc_task, OS_NULL, ADC_STK_SIZE, ADC_TASK_PRIO);      
    OS_ASSERT(ADC_Handler);
    os_task_startup(ADC_Handler);
	 
	 NFC_Handler = os_task_create("nfc_task", nfc_task, OS_NULL, NFC_STK_SIZE, NFC_TASK_PRIO);      
    OS_ASSERT(NFC_Handler);
    os_task_startup(NFC_Handler);
	 
	 LEND_Handler = os_task_create("lend_task", lend_task, OS_NULL, LEND_STK_SIZE, LEND_TASK_PRIO);      
    OS_ASSERT(LEND_Handler);
    os_task_startup(LEND_Handler);
	 
	 TIMER_PERIODIC = os_timer_create("my_device_sensor_d", my_device_sensor_d,OS_NULL, 100, OS_TIMER_FLAG_PERIODIC);
	 OS_ASSERT_EX(OS_NULL != TIMER_PERIODIC, "timer create err\r\n");
	 os_timer_start(TIMER_PERIODIC);
	 
	 SERIAL_Handler = os_task_create("serial_task",          /* 设置任务的名称 */
                                    serial_task,            /* 设置任务函数 */
                                    OS_NULL,                /* 任务传入的参数 */
                                    SERIAL_STK_SIZE,        /* 设置任务堆栈 */
                                    SERIAL_TASK_PRIO);      /* 设置任务的优先级 */
    OS_ASSERT(SERIAL_Handler);
    os_task_startup(SERIAL_Handler);                        /* 任务开始 */
    return 0;
}
