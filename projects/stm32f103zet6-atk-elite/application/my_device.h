#include <board.h>

#define LAMP_PIN				  0//灯、与继电器串联

#define MOTOR_PIN				 99//PG3排风扇
#define ALARM_PIN				101//PG5警报
#define SOUND_SENSOR_PIN		103//PG7声音传感器
#define LIGHT_SENSOR_D_PIN		 38//PC6光照传感器D（数字量）
#define MQ2_SENSOR_D_PIN		 40//PC8烟雾传感器D

#define BOX1_SENSOR_PIN			 98//PG2器件盒光电传感器1
#define BOX2_SENSOR_PIN			100//PG4器件盒光电传感器2
#define TOUCH_SENSOR_PIN		102//PG6触摸开关
#define HUMAN_SENSOR_PIN		104//PG8人体存在传感器

#define RELAY1_PIN				 35//PC3继电器1
#define RELAY2_PIN				 33//PC1继电器2
								 
#define LIGHT_SENSOR_A_PIN		 34//PC2光照传感器A（模拟量）!!  未使用
#define MQ2_SENSOR_A_PIN		 32//PC0烟雾传感器A!!
#define LIGHT_SENSOR_B_PIN		 88//PF8板载光照传感器

//#define POWER_SENSOR1_A_PIN		 36//PC4功率传感器  //P=U1*(U2-U1)/U1
#define POWER_SENSOR1_A_PIN		 34//PC2功率传感器
#define POWER_SENSOR2_A_PIN		 37//PC5功率传感器

//extern struct device_state device_state;
typedef struct{
	unsigned int	sound_num;
	unsigned int	light_num;
	unsigned int	mq2_num; //警报数量
	unsigned char  lamp;		//灯(数字量)
			 int  temperature;	//温度 单位度
			 int  humidity;		//湿度 单位百分比
	unsigned int  power;		//功率  单位mW
	unsigned char  sound;		//噪声(数字量)
	unsigned char light_d;		//光强（数字量）
	unsigned int  light_b;		//板载光强  单位Lx
	unsigned int  light;		//光强  
	unsigned char mq2_d;		//烟雾（数字量）
	unsigned int  mq2;			//烟雾
	unsigned char box1;			//器件盒1（数字量）
	unsigned char box2;			//器件盒2（数字量）
	unsigned char touch;		//触摸开关（数字量）
	unsigned char human;		//人体存在（数字量）
	unsigned char door;			//门（数字量）
	
	unsigned char motor;		//电机状态
	unsigned char alarm;		//警报状态
	unsigned char relay1;		//继电器1状态
	unsigned char relay2;		//继电器2状态
	
	char nfc_id_now[9];//当前nfc的id
	char nfc_id_last[9];//上一次nfc的id
//	unsigned char nfc_id_now[9];//当前nfc的id
//	unsigned char nfc_id_last[9];//上一次nfc的id
	
}my_device;

extern my_device device_state;

extern void my_device_init(void);
extern void my_device_cmd_control(unsigned char cmd);
extern void my_device_sensor_d(void *parameter);
