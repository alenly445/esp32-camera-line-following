#pragma once
#include "esp_err.h"

/* 屏幕上显示的小车运动判别。 */
typedef enum {
    CAR_DISPLAY_STRAIGHT,
    CAR_DISPLAY_DETECTED_LEFT,
    CAR_DISPLAY_DETECTED_RIGHT,
    CAR_DISPLAY_LOST_FORWARD,
    CAR_DISPLAY_TURNING_LEFT,
    CAR_DISPLAY_TURNING_RIGHT,
    CAR_DISPLAY_NEW_LINE,
    CAR_DISPLAY_MOVE_LEFT,
    CAR_DISPLAY_STOP,
} car_display_state_t;

/* 初始化ST7735屏幕及独立刷新任务。 */
esp_err_t car_display_init(void);

/* 更新屏幕需要显示的三轮PWM和运动判别，不阻塞巡线任务。 */
void car_display_set(int left_pwm, int back_pwm, int right_pwm,
                     car_display_state_t state);
