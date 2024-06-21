#include "rc.h"


void right_wheel_dir(uint8_t dir) {
    if (dir == FORWARD) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
    }
    else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
    }
    return;
}

void left_wheel_dir(uint8_t dir) {
    if (dir == FORWARD) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
    }
    else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
    }
    return;
}

void right_wheel_inc_speed(uint16_t speed) { 
    uint32_t tmp_speed = htim2.Instance->CCR1 + speed;
    if (tmp_speed > RC_MAX_SPEED) tmp_speed = RC_MAX_SPEED;
    htim2.Instance->CCR1 = tmp_speed;
    return;
}

void left_wheel_inc_speed(uint16_t speed) { 
    uint32_t tmp_speed = htim4.Instance->CCR2 + speed;
    if (tmp_speed > RC_MAX_SPEED) tmp_speed = RC_MAX_SPEED;
    htim4.Instance->CCR2 = tmp_speed;
    return;
}

void left_wheel_set_speed(uint16_t speed) {
    if (speed > RC_MAX_SPEED) speed = RC_MAX_SPEED;
    htim4.Instance->CCR2 = speed;
    return;
}

void right_wheel_set_speed(uint16_t speed) {
    if (speed > RC_MAX_SPEED) speed = RC_MAX_SPEED;
    htim2.Instance->CCR1 = speed;
    return;
}

uint16_t right_wheel_get_speed(void) {
    return htim2.Instance->CCR1;
}

uint16_t left_wheel_get_speed(void) {
    return htim4.Instance->CCR2;
}

uint8_t wheel_stop() {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
	left_wheel_set_speed(0);
	right_wheel_set_speed(0);
	return STATE_STOP;
}

uint8_t go_forward(uint16_t speed) {
	left_wheel_dir(FORWARD);
	right_wheel_dir(FORWARD);
	left_wheel_set_speed(speed);
	right_wheel_set_speed(speed);
	return STATE_FORWARD;
}

uint8_t go_backward(uint16_t speed) {
	left_wheel_dir(BACKWARD);
	right_wheel_dir(BACKWARD);
	left_wheel_set_speed(speed);
	right_wheel_set_speed(speed);
	return STATE_BACKWARD;
}

uint8_t turn_left(int16_t speed_l, int16_t speed_r) {
    uint8_t direction = FORWARD;
    uint16_t speed_l_abs = (uint16_t)abs(speed_l); // 절대값을 구한다.
    if (speed_l < 0) {
        direction = BACKWARD;     // 후진으로 바꾼다.
    }
	left_wheel_dir(direction);
	right_wheel_dir(FORWARD);
	left_wheel_set_speed(speed_l_abs);
	right_wheel_set_speed(speed_r);
	return STATE_TURN_LEFT;
}

uint8_t turn_right(int16_t speed_l, int16_t speed_r) {
    uint8_t direction = FORWARD;
    uint16_t speed_r_abs = (uint16_t)abs(speed_r); // 절대값을 구한다.
    if (speed_r < 0) {
        direction = BACKWARD;     // 후진으로 바꾼다.
    }
	left_wheel_dir(FORWARD);
	right_wheel_dir(direction);
	left_wheel_set_speed(speed_l);
	right_wheel_set_speed(speed_r_abs);
	return STATE_TURN_RIGHT;
}