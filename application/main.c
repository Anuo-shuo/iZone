/* ����
include
define
int
�ź���
TASK���ü���������
����
������
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

//�豸
#include "my_device.h"

//SHT40
//#include "sht40/sht4x.h"

//�ϴ�����Ϣ
#include "upload_msg.h"

//������
#include <stdio.h>
#include <sensors/sensor.h>


#define RECV_BUF_LEN     (1024)
#define TEST_MODULE_NAME "esp8266"
#define AP_SSID          "201RIGHT"
#define AP_PASSWORD      "erlingyiyou"

//MQTT�յ��Ŀ������ݣ�1�ֽڣ�
char mqtt_receive_data;

//������sensor����
//������name�� temp_aht10 humi_aht10 
struct os_sensor_data sensor_data_temp;
struct os_sensor_data sensor_data_humi;


//�豸����
os_device_t *adc_1;
os_device_t *adc_2;
os_device_t *adc_3;
os_device_t *os_uart_3;
os_pwm_device_t *pwm_dev = OS_NULL;
os_timer_t *TIMER_PERIODIC = OS_NULL;
//os_device_t *sensor_temp;
//os_device_t *sensor_humi;
/********************************************************************************************************/
/* ���ݶ��� */
//��ǰ�豸״̬
//struct device_state{
//	unsigned int  temperature;	//�¶�
//	unsigned int  humidity;		//ʪ��
//	unsigned int  power;		//����
//	unsigned char  sound;		//����(������)
//	unsigned char light_d;		//��ǿ����������
//	unsigned int  light_b;		//���ع�ǿ
//	unsigned int  light;		//��ǿ
//	unsigned char mq2_d;		//������������
//	unsigned int  mq2;			//����
//	unsigned char box1;			//������1����������
//	unsigned char box2;			//������2����������
//	unsigned char touch;		//�������أ���������
//	unsigned char human;		//������ڣ���������
//	unsigned char door;			//�ţ���������
//	
//	unsigned char motor;		//���״̬
//	unsigned char alarm;		//����״̬
//	unsigned char relay1;		//�̵���1״̬
//	unsigned char relay2;		//�̵���2״̬
//	
//}device_state;

//�����豸
struct device_control{

	unsigned char door;			//��
	//�ĺ�һ
//	unsigned char motor;		//���
//	unsigned char alarm;		//����
//	unsigned char relay1;		//�̵���1
//	unsigned char relay2;		//�̵���2
	//�ĺ�һ
	unsigned char d_switch; 	//���������豸

	
}device_control;

struct lcd_conf{

	int page;			//ҳ
	int cursor;			//ָ��
	
	
}lcd_conf;



/********************************************************************************************************/
/* �ź������� */
static os_sem_t* sem_ota;	//ota�ź���
static os_sem_t* sem_lcd;	//lcd�ź���
//extern os_sem_t* cmd_device;	//cmd_device�ź���
os_sem_t* sem_cmd_device;	//cmd_device�ź���
os_sem_t* sem_lend_device;	//lend_device�ź���
//os_sem_t* sem_auto_alarm;	//auto_alarm�ź���
os_sem_t* sem_door;			//door�ź���
os_sem_t* sem_nfc;			//nfc�ź���
os_sem_t* sem_lend;			//��������ź���
os_sem_t* sem_lend_post;			//��������ź���
/* oneOS����************************************************************************************** */
/* MQTT_TASK ���� ����
 * ����: ������ �������ȼ� ��ջ��С ��������
 */

//MQTT��������
#define MQTT_TASK_PRIO    6              /* �������ȼ� */
#define MQTT_STK_SIZE     2048           /* �����ջ��С */
os_task_t *MQTT_Handler;                 /* ������ƿ� */
void mqtt_task(void *parameter); 		  /* ������ */


//KEY��������
#define KEY_TASK_PRIO    7              /* �������ȼ� */
#define KEY_STK_SIZE     512            /* �����ջ��С */
os_task_t *KEY_Handler;                 /* ������ƿ� */
void key_task(void *parameter); 		 /* ������ */


//OTA��������
#define OTA_TASK_PRIO    17             /* �������ȼ� */
#define OTA_STK_SIZE     2048           /* �����ջ��С */
os_task_t *OTA_Handler;                 /* ������ƿ� */
void ota_task(void *parameter); 		 /* ������ */

//LCD��������
#define LCD_TASK_PRIO    9              /* �������ȼ� */
#define LCD_STK_SIZE     1024           /* �����ջ��С */
os_task_t *LCD_Handler;                 /* ������ƿ� */
void lcd_task(void *parameter);			 /* ������ */

//CMD_DEVICE��������
#define CMD_DEVICE_TASK_PRIO    5              /* �������ȼ� */
#define CMD_DEVICE_STK_SIZE     512            /* �����ջ��С */
os_task_t *CMD_DEVICE_Handler;                 /* ������ƿ� */
void cmd_device(void *parameter); 				  /* ������ */

//LEND_DEVICE��������
#define LEND_DEVICE_TASK_PRIO    14            /* �������ȼ� */
#define LEND_DEVICE_STK_SIZE     512           /* �����ջ��С */
os_task_t *LEND_DEVICE_Handler;                /* ������ƿ� */
void lend_device(void *parameter); 			  /* ������ */
//AUTO_ALARM��������
#define AUTO_ALARM_TASK_PRIO    11             /* �������ȼ� */
#define AUTO_ALARM_STK_SIZE     512            /* �����ջ��С */
os_task_t *AUTO_ALARM_Handler;                 /* ������ƿ� */
void auto_alarm(void *parameter); 				  /* ������ */

//MQTT_CMD��������
#define MQTT_CMD_TASK_PRIO    10             /* �������ȼ� */
#define MQTT_CMD_STK_SIZE     512            /* �����ջ��С */
os_task_t *MQTT_CMD_Handler;                 /* ������ƿ� */
void mqtt_cmd(void *parameter);				   /* ������ */

//RC522��������
#define RC522_TASK_PRIO    18             /* �������ȼ� */
#define RC522_STK_SIZE     512            /* �����ջ��С */
os_task_t *RC522_Handler;                 /* ������ƿ� */
void rc522_task(void *parameter);		   /* ������ */

//DOOR��������
#define DOOR_TASK_PRIO    17             /* �������ȼ� */
#define DOOR_STK_SIZE     512            /* �����ջ��С */
os_task_t *DOOR_Handler;                 /* ������ƿ� */
void door_task(void *parameter);		  /* ������ */

//ADC��������
#define ADC_TASK_PRIO    11             /* �������ȼ� */
#define ADC_STK_SIZE     1024           /* �����ջ��С */
os_task_t *ADC_Handler;                 /* ������ƿ� */
void adc_task(void *parameter); 		 /* ������ */

//NFC��������
#define NFC_TASK_PRIO    11             /* �������ȼ� */
#define NFC_STK_SIZE     1024           /* �����ջ��С */
os_task_t *NFC_Handler;                 /* ������ƿ� */

//LEND POST��������
#define LEND_TASK_PRIO    10             /* �������ȼ� */
#define LEND_STK_SIZE     512            /* �����ջ��С */
os_task_t *LEND_Handler;                 /* ������ƿ� */
void lend_cmd(void *parameter); 		  /* ������ */

// SERIAL_TASK ���� ����
#define SERIAL_TASK_PRIO        18       /* �������ȼ� */
#define SERIAL_STK_SIZE         512      /* �����ջ��С */
os_task_t *SERIAL_Handler;               /* ������ƿ� */
void serial_task(void *parameter);      /* ������ */

#define USART2_MAX_RX_LEN    20                 /* �����ջ����ֽ��� */
os_uint8_t USART2_RX_BUF[USART2_MAX_RX_LEN];    /* ���ջ���,���USART2_MAX_RECV_LEN���ֽ�. */


os_uint8_t k_back=0;		//���·��ؼ�
os_uint8_t k_ok=0;		//����ȷ�ϼ�
char is_using_lend=0;	//lendģʽ
void nfc_task(void *parameter); //rc522���
/********************************************************************************************************/
/* ������Ϣ */
//�������upload_msg.h��
extern struct os_mq mqtts_mq;
/********************************************************************************************************/
/*�Ӻ���*/

/**
 * @brief       ����2�豸��ʼ�������ڴ�CI1122�������ݣ�
 * @param       ��
 * @retval      �����豸���
 */
os_device_t *os_usart_2_init()
{
    os_device_t *os_uart_2;
    os_uart_2 = os_device_find("uart2");                            /* Ѱ���豸 */
    os_device_open(os_uart_2);                                      /* ���豸 */
    struct serial_configure config = OS_SERIAL_CONFIG_DEFAULT;      /* ����Ĭ�� */
    os_device_control(os_uart_2, OS_DEVICE_CTRL_CONFIG, &config);   /* �����豸 */
    
    return os_uart_2;
}

/**
 * @brief       serial_task����CI1122������������ݣ�
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
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
 * @brief       ��ʼ������Ļ molink��
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
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
    /* �ֶ�����ģ�� */
   mo_object_t *test_module = OS_NULL;
   mo_object_t *temp_module = OS_NULL;
   os_err_t ret = OS_EOK;
   lcd_show_string(30, 10, 200, 16, 16, "iZone V1.0", RED);
	lcd_show_string(30, 30, 200, 16, 16, "Welcome!!!", DARKBLUE);

	
	
	lcd_show_string(30, 70, 200, 16, 16, "Connnecting WiFi...", DARKBLUE);
   os_uart_3 = os_device_find("uart3");  /* Ѱ���豸 */
   struct serial_configure config = OS_SERIAL_CONFIG_DEFAULT;   /* ����Ĭ�� */
   config.baud_rate = BAUD_RATE_115200;                         /* ���ò����� */
   os_device_control(os_uart_3, OS_DEVICE_CTRL_CONFIG, &config);/* �����豸 */
    
    /* hardware reset esp8266 */
   esp8266_hw_rst(ESP8266_RST_PIN_NUM);
   os_task_msleep(3000);

   mo_parser_config_t parser_config = {.parser_name   = TEST_MODULE_NAME,
                                       .parser_device = os_uart_3,
                                       .recv_buff_len = RECV_BUF_LEN};
    
   test_module = mo_create("esp8266", MODULE_TYPE_ESP8266, &parser_config); /* ����molink */
   OS_ASSERT(OS_NULL != test_module);

   temp_module = mo_get_default();    /* ��ȡmolink��� */
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
 * @brief       �������
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
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
        
        if (key == WKUP_PRES)   		//����
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
        else if(key == KEY1_PRES)	//ȷ��
		  {
			  if(lcd_conf.page==3) k_ok=1; 
			  else{
				lcd_conf.page=lcd_conf.cursor;
				lcd_conf.cursor=0;
			   os_sem_post(sem_lcd);
			  }  
		  }
		  else if(key == KEY0_PRES)	//ָ���һ
		  {
			  lcd_conf.cursor++;
			  if(lcd_conf.cursor>4)lcd_conf.cursor=1;
			  os_sem_post(sem_lcd);		  
		  }	  
        os_task_msleep(10);		
    }

}

/**
 * @brief       LCDˢ��
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
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
					   os_sem_post(sem_lend);//����OTA�ź���
					   break;
				  }
												  
				  case(4):{
				   	lcd_clear(WHITE);
					   lcd_show_string(30, 10, 200, 16, 16, "OTA", BLACK);
					   os_sem_post(sem_ota);//����OTA�ź���
					   break;
				  }
				  
			  }
			  if(lcd_conf.page==0&&lcd_conf.cursor>0)lcd_fill_circle(15, lcd_conf.cursor*20+18,5,LIGHTGREEN);
		 }
	 }
    
}

/**
 * @brief       OTA����
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
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
 * @brief       ONENET MQTT���ͣ�publish��Ϣ̫�� �ֳ���6�Σ�
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
 */
static void mqtt_task(void *parameter)
{
	
	    /* ����onenet mqtt */
    onenet_mqtts_device_start();
	


	//MQTT���պ�ִ��onenet_mqtts.c��submessage_cmd_request_arrived_handler
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
        if (onenet_mqtts_device_is_connected() == 1) /* �ж��豸�Ƿ����� */
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
			FILL_MSG1;//��upload_msg.h��
            pub_msg = pub_buf1;
            pub_msg_len = strlen(pub_msg);
            memset(&mq_msg, 0x00, sizeof(mq_msg));		//����
            mq_msg.topic_type = DATA_POINT_TOPIC;		//��������
            memcpy(mq_msg.data_buf, pub_msg, pub_msg_len);	//�� pub_msg��ֵ��������Ϣ���е���Ϣ��������
            mq_msg.data_len = pub_msg_len;				//��Ϣ����
//			os_kprintf("msg1_len %d \n", mq_msg.data_len);
            rc = os_mq_send(&mqtts_mq, (void *)&mq_msg, sizeof(mq_msg_t), 0);  /* �������� */
            if (rc != OS_EOK)
            {
                os_kprintf("\r\n 1 mqtts_device_messagequeue_send ERR\r\n");
            }
				os_task_msleep(950);
			/***********************************************/
				id++;
				FILL_MSG2;//��upload_msg.h��
            pub_msg = pub_buf2;
            pub_msg_len = strlen(pub_msg);
            memset(&mq_msg, 0x00, sizeof(mq_msg));		//����
            mq_msg.topic_type = DATA_POINT_TOPIC;		//��������
            memcpy(mq_msg.data_buf, pub_msg, pub_msg_len);	//�� pub_msg��ֵ��������Ϣ���е���Ϣ��������
            mq_msg.data_len = pub_msg_len;				//��Ϣ����
//				 os_kprintf("msg2_len %d \n", mq_msg.data_len);
            rc = os_mq_send(&mqtts_mq, (void *)&mq_msg, sizeof(mq_msg_t), 0);  /* �������� */
            if (rc != OS_EOK)
            {
                os_kprintf("\r\n 2 mqtts_device_messagequeue_send ERR\r\n");
            }
				os_task_msleep(950);
			/***********************************************/
				id++;
				FILL_MSG3;//��upload_msg.h��
            pub_msg = pub_buf3;
            pub_msg_len = strlen(pub_msg);
            memset(&mq_msg, 0x00, sizeof(mq_msg));		//����
            mq_msg.topic_type = DATA_POINT_TOPIC;		//��������
            memcpy(mq_msg.data_buf, pub_msg, pub_msg_len);	//�� pub_msg��ֵ��������Ϣ���е���Ϣ��������
            mq_msg.data_len = pub_msg_len;				//��Ϣ����
//				 os_kprintf("msg3_len %d \n", mq_msg.data_len);
            rc = os_mq_send(&mqtts_mq, (void *)&mq_msg, sizeof(mq_msg_t), 0);  /* �������� */
            if (rc != OS_EOK)
            {
                os_kprintf("\r\n 3 mqtts_device_messagequeue_send ERR\r\n");
            }
				os_task_msleep(950);
			/***********************************************/
				id++;
				FILL_MSG4;//��upload_msg.h��
            pub_msg = pub_buf4;
            pub_msg_len = strlen(pub_msg);
            memset(&mq_msg, 0x00, sizeof(mq_msg));		//����
            mq_msg.topic_type = DATA_POINT_TOPIC;		//��������
            memcpy(mq_msg.data_buf, pub_msg, pub_msg_len);	//�� pub_msg��ֵ��������Ϣ���е���Ϣ��������
            mq_msg.data_len = pub_msg_len;				//��Ϣ����
//				 os_kprintf("msg4_len %d \n", mq_msg.data_len);
            rc = os_mq_send(&mqtts_mq, (void *)&mq_msg, sizeof(mq_msg_t), 0);  /* �������� */
            if (rc != OS_EOK)
            {
                os_kprintf("\r\n 4 mqtts_device_messagequeue_send ERR\r\n");
            }
				os_task_msleep(950);
			/***********************************************/
				id++;
				FILL_MSG5;//��upload_msg.h��
            pub_msg = pub_buf5;
            pub_msg_len = strlen(pub_msg);
            memset(&mq_msg, 0x00, sizeof(mq_msg));		//����
            mq_msg.topic_type = DATA_POINT_TOPIC;		//��������
            memcpy(mq_msg.data_buf, pub_msg, pub_msg_len);	//�� pub_msg��ֵ��������Ϣ���е���Ϣ��������
            mq_msg.data_len = pub_msg_len;				//��Ϣ����
//				 os_kprintf("msg5_len %d \n", mq_msg.data_len);
            rc = os_mq_send(&mqtts_mq, (void *)&mq_msg, sizeof(mq_msg_t), 0);  /* �������� */
            if (rc != OS_EOK)
            {
                os_kprintf("\r\n 5 mqtts_device_messagequeue_send ERR\r\n");
            }
				os_task_msleep(950);
        
		  
			/***********************************************/
				if(device_state.light_num>0||device_state.sound_num>0||device_state.mq2_num>0){
					id++;
					FILL_MSG6;//��upload_msg.h��
					pub_msg = pub_buf6;
					pub_msg_len = strlen(pub_msg);
					memset(&mq_msg, 0x00, sizeof(mq_msg));		//����
					mq_msg.topic_type = DATA_POINT_TOPIC;		//��������
					memcpy(mq_msg.data_buf, pub_msg, pub_msg_len);	//�� pub_msg��ֵ��������Ϣ���е���Ϣ��������
					mq_msg.data_len = pub_msg_len;				//��Ϣ����
	//				 os_kprintf("msg5_len %d \n", mq_msg.data_len);
					rc = os_mq_send(&mqtts_mq, (void *)&mq_msg, sizeof(mq_msg_t), 0);  /* �������� */
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
 * @brief       MQTT���յ�0x01��ִ��task1
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
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
 * @brief       �������
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
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
 * @brief       �豸����(�Զ����ƺ��ϴ�����)
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
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
 * @brief       ����ϴ�
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
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
			  
			 
				FILL_MSG_LEND;//��upload_msg.h��
            pub_msg = pub_buf_lend;
            pub_msg_len = strlen(pub_msg);
            memset(&mq_msg, 0x00, sizeof(mq_msg));		//����
            mq_msg.topic_type = DATA_POINT_TOPIC;		//��������
            memcpy(mq_msg.data_buf, pub_msg, pub_msg_len);	//�� pub_msg��ֵ��������Ϣ���е���Ϣ��������
            mq_msg.data_len = pub_msg_len;				//��Ϣ����
				os_kprintf("msg_nfc: %s \n", mq_msg.data_buf);
            rc = os_mq_send(&mqtts_mq, (void *)&mq_msg, sizeof(mq_msg_t), 0);  /* �������� */
            if (rc != OS_EOK)
            {
                os_kprintf("\r\n NFC mqtts_device_messagequeue_send ERR\r\n");
            }
				os_task_msleep(950);
		 } 
		
	 }
	
}


/**
 * @brief       nfc��ȡ��ID��publish ID�뿪���Ų���
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
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
			 
				FILL_MSG_NFC;//��upload_msg.h��
            pub_msg = pub_buf_nfc;
            pub_msg_len = strlen(pub_msg);
            memset(&mq_msg, 0x00, sizeof(mq_msg));		//����
            mq_msg.topic_type = DATA_POINT_TOPIC;		//��������
            memcpy(mq_msg.data_buf, pub_msg, pub_msg_len);	//�� pub_msg��ֵ��������Ϣ���е���Ϣ��������
            mq_msg.data_len = pub_msg_len;				//��Ϣ����
				os_kprintf("msg_nfc_nfc_task: %s \n", mq_msg.data_buf);
            rc = os_mq_send(&mqtts_mq, (void *)&mq_msg, sizeof(mq_msg_t), 0);  /* �������� */
            if (rc != OS_EOK)
            {
                os_kprintf("\r\n NFC mqtts_device_messagequeue_send ERR\r\n");
            }
				os_task_msleep(950);
			 
		 } 
	 }
	
}
/**
 * @brief       ģ��������봫����
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
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
		
		//sensor �¶�
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
		  //sensor ʪ��
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
		  
	
		//���ʼ��
		// os_device_read_nonblock( adc_1, ADC_CHANNEL_12,&device_state.light,sizeof(device_state.light));//adc1�޷�ʹ��
		os_device_read_nonblock( adc_2, 12,&power1,sizeof(power1));//POWER_SENSOR1_A_PIN
		os_device_read_nonblock( adc_2, ADC_CHANNEL_15,&power2,sizeof(power2));//POWER_SENSOR1_B_PIN
		 device_state.power=16*power1*(power1-power2)/30/1000;
//		  os_kprintf("\r\n power---->%d,%d,%d\r\n",power1,power2,device_state.power);
//		  //MQ2��Ч
//		 os_device_read_nonblock( adc_2, ADC_CHANNEL_10,&device_state.mq2,sizeof(device_state.mq2));
//		   os_kprintf("\r\n ch10---->%d\r\n",device_state.mq2);
		//���ع��գ�����֤��
		os_device_read_nonblock( adc_3, ADC_CHANNEL_6,&light_b,sizeof(light_b));
		device_state.light_b=(int)(0.06728*(3300-light_b));	 
		  
//		os_kprintf("------->%d\r\n",device_state.light_b);
		  
		os_task_msleep(1000);
	 }
	
}
/**
 * @brief       �ſ���
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
 */
static void door_task(void *parameter)
{
	
	
	parameter=parameter;
	
	os_uint32_t period = 20000000;  
   os_uint32_t close_pulse  = 1400000; //1500000  0�� 1850000 63��     
	os_uint32_t open_pulse  = 1900000;
   os_uint32_t channel= 3;//PB8
	//���ط�����+��
	pwm_dev =  (os_pwm_device_t *)os_device_find("pwm_tim4");
	    if (pwm_dev == OS_NULL)
    {
        os_kprintf("pwm sample run failed! can't find device!\n");
       // return OS_ERROR;
    }
	//00 �ر�
	//01 ����
	//02 �ȿ����
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
 * @brief       TOUCH�жϺ���
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
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
 * @brief       MQTT���յ����ݼ��
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
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
 * @brief       RC522_TASK�Ӻ�������ʾNFC��ID
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
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
 * @brief       RC522��ȡNFC��ID
 * @param       parameter : �������(δ�õ�)
 * @retval      ��
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
					ShowID(SelectedSnr);//��ȡID
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

	//��ʼ��
    all_init();
	
  /* USER CODE BEGIN 2 */
	
	//�ź�������
	 sem_ota = os_sem_create("sem_ota", 0,1);
	 sem_lcd = os_sem_create("sem_lcd", 0,1);
	 sem_cmd_device= os_sem_create("sem_cmd_device", 0,1);//cmd_device�ź���
    //sem_lend_device= os_sem_create("sem_lend_device", 0,1);//lend_device�ź���
    //sem_auto_alarm= os_sem_create("sem_auto_alarm", 0,1);//auto_alarm�ź���
	 sem_lend= os_sem_create("sem_lend", 0,1);//�������
	 sem_lend_post= os_sem_create("sem_lend_post", 0,1);//�ϴ�����
	 sem_door = os_sem_create("sem_door", 0,1);//door
	 sem_nfc = os_sem_create("sem_nfc", 0,1);//nfc
	// OS_ASSERT_EX(OS_NULL != sem_ota, "sem_ota create err!\r\n");	
	
    /* ���жϣ�������ģʽ���ص�������Ϊtouch_irq */
    os_pin_attach_irq(TOUCH_SENSOR_PIN, PIN_IRQ_MODE_RISING, touch_irq, OS_NULL);
    /* ʹ���ж� */
    os_pin_irq_enable(TOUCH_SENSOR_PIN, PIN_IRQ_ENABLE);
	 
	//���񴴽�
    MQTT_Handler = os_task_create(  "mqtt_task",   /* ������������� */
                                  mqtt_task,     /* ���������� */
                                  OS_NULL,              /* ������Ĳ��� */
                                  MQTT_STK_SIZE,        /* ���������ջ */
                                  MQTT_TASK_PRIO);      /* ������������ȼ� */
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
	 
	 SERIAL_Handler = os_task_create("serial_task",          /* ������������� */
                                    serial_task,            /* ���������� */
                                    OS_NULL,                /* ������Ĳ��� */
                                    SERIAL_STK_SIZE,        /* ���������ջ */
                                    SERIAL_TASK_PRIO);      /* ������������ȼ� */
    OS_ASSERT(SERIAL_Handler);
    os_task_startup(SERIAL_Handler);                        /* ����ʼ */
    return 0;
}
