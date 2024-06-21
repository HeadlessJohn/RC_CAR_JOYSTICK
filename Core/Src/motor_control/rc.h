
#ifndef RC_H
#define RC_H

#include "stm32f4xx_hal.h"
#include "tim.h"
#include <stdlib.h> 

#define FORWARD      1
#define BACKWARD     0
#define RIGHT_WHEEL_SPEED   htim2.Instance->CCR1
#define LEFT_WHEEL_SPEED    htim4.Instance->CCR2

#define RC_MAX_SPEED		1000
#define RC_MIN_SPEED		100

#define STATE_STOP				0
#define STATE_FORWARD			1
#define STATE_BACKWARD			2
#define STATE_TURN_LEFT 		3
#define STATE_TURN_RIGHT		4
#define STATE_INC_FOWARD_SPEED	5
#define STATE_DEC_FOWARD_SPEED	6
#define STATE_INC_TURN_SPEED	7
#define STATE_DEC_TURN_SPEED	8

#define LEFT 	0
#define RIGHT 	1

void left_wheel_dir(uint8_t dir);
void right_wheel_dir(uint8_t dir);
void left_wheel_inc_speed(uint16_t speed);
void right_wheel_inc_speed(uint16_t speed);
void left_wheel_set_speed(uint16_t speed);
void right_wheel_set_speed(uint16_t speed);
uint16_t left_wheel_get_speed(void);
uint16_t right_wheel_get_speed(void);
uint8_t wheel_stop();
uint8_t go_forward(uint16_t speed);
uint8_t go_backward(uint16_t speed);
uint8_t turn_left(int16_t speed_l, int16_t speed_r);
uint8_t turn_right(int16_t speed_l, int16_t speed_r);

#endif // RC_H