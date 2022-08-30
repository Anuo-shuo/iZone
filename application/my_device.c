#include "my_device.h"

my_device device_state;

void my_device_init(void){
	os_pin_mode(MOTOR_PIN			, PIN_MODE_OUTPUT);
		os_pin_write(MOTOR_PIN, PIN_HIGH);
	os_pin_mode(ALARM_PIN			, PIN_MODE_OUTPUT);
		os_pin_write(ALARM_PIN, PIN_HIGH);
	os_pin_mode(RELAY1_PIN			, PIN_MODE_OUTPUT);
	os_pin_mode(RELAY2_PIN			, PIN_MODE_OUTPUT);
	
	os_pin_mode(SOUND_SENSOR_PIN	, PIN_MODE_INPUT);
	os_pin_mode(LIGHT_SENSOR_D_PIN	, PIN_MODE_INPUT);
	os_pin_mode(MQ2_SENSOR_D_PIN	, PIN_MODE_INPUT);
	os_pin_mode(BOX1_SENSOR_PIN		, PIN_MODE_INPUT);
	os_pin_mode(BOX2_SENSOR_PIN		, PIN_MODE_INPUT);
	os_pin_mode(TOUCH_SENSOR_PIN	, PIN_MODE_INPUT);
	os_pin_mode(HUMAN_SENSOR_PIN	, PIN_MODE_INPUT);
	
}

void my_device_cmd_control(unsigned char cmd){
	unsigned char cmd_h8 = (cmd & 0xf0) >> 4;
	unsigned char cmd_l8 = cmd & 0x0f;
	switch(cmd_h8){
		case 6:{
			if(cmd_l8==0) {
				os_pin_write(MOTOR_PIN, PIN_HIGH);
				device_state.motor = 0;
			}
			else {
				os_pin_write(MOTOR_PIN, PIN_LOW);
				device_state.motor = 1;
			}
			break;
		}
		case 2:{
			if(cmd_l8==0) {
				os_pin_write(ALARM_PIN, PIN_HIGH);
				device_state.alarm = 0;
			}
			else {
				os_pin_write(ALARM_PIN, PIN_LOW);
				device_state.alarm = 1;
			}
			break;
		}
		case 3:{
			
			switch(cmd_l8){
				case 0:{
					os_pin_write(RELAY1_PIN, PIN_LOW);
					device_state.relay1 = 0;					
					break;
				}
				case 1:{
					os_pin_write(RELAY1_PIN, PIN_HIGH);
					device_state.relay1 = 1;
					break;
				}
				case 2:{
					os_pin_write(RELAY2_PIN, PIN_LOW);
					device_state.relay2 = 0;
					break;
				}
				case 3:{
					os_pin_write(RELAY2_PIN, PIN_HIGH);
					device_state.relay2 = 1;
					break;
				}
			}
			break;
		}
		case 4:{
			if(cmd_l8==0) {
				os_pin_write(RELAY1_PIN, PIN_LOW);
				device_state.lamp = 0;
			}
			else {
				os_pin_write(RELAY1_PIN, PIN_HIGH);
				device_state.lamp = 1;
			}
			break;
		}
	
	}
}

void my_device_sensor_d(void *parameter){
	device_state.sound = !os_pin_read(SOUND_SENSOR_PIN);
	
	device_state.light_d = !os_pin_read(LIGHT_SENSOR_D_PIN);
	
	device_state.mq2_d = os_pin_read(MQ2_SENSOR_D_PIN);
	
	device_state.box1 = !os_pin_read(BOX1_SENSOR_PIN);
	
	device_state.box2 = !os_pin_read(BOX2_SENSOR_PIN);
	
//	device_state.touch = os_pin_read(TOUCH_SENSOR_PIN);
	
	device_state.human = os_pin_read(HUMAN_SENSOR_PIN);
	
}
