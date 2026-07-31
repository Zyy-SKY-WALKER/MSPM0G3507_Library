/**
 * @file    test_config.h
 * @brief   Select exactly one application test mode.
 */

#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#define TEST_MODE_NORMAL             (0)
#define TEST_MODE_ILI9341            (1)
#define TEST_MODE_MOTOR              (2)
#define TEST_MODE_ENCODER            (3)
#define TEST_MODE_SPEED_PID          (4)
#define TEST_MODE_IMU_UART           (5)
#define TEST_MODE_ODOMETRY           (6)
#define TEST_MODE_GRAY_SENSOR        (7)
#define TEST_MODE_VOFA_SPEED         (8)
#define TEST_MODE_LINE_TRACKER       (9)
#define TEST_MODE_GPIO_OUTPUT        (10)
#define TEST_MODE_GPIO_INPUT         (11)
#define TEST_MODE_CONTROL_SCHEDULER  (12)
#define TEST_MODE_SERVO              (13)
#define TEST_MODE_CHASSIS_MOTION     (14)
#define TEST_MODE_LINE_FOLLOW_REAL   (15)
#define TEST_MODE_STEPPER            (16)
#define TEST_MODE_XPT2046            (17)
#define TEST_MODE_TOUCH_GIMBAL       (18)
#define TEST_MODE_ILI9341_SNOW       (19)
#define TEST_MODE_ILI9341_DISSOLVE   (20)
#define TEST_MODE_ILI9341_MOSAIC     (21)
#define TEST_MODE_ILI9341_FADE       (22)
#define TEST_MODE_ILI9341_NEON       (23)
#define TEST_MODE_ILI9341_RADIAL     (24)
#define TEST_MODE_VISION_UART        (25)
#define TEST_MODE_MPU6500            (26)
#define TEST_MODE_MPU6500_YAW_TURN   (27)
#define TEST_MODE_MPU6500_YAW_TURN_CLOSED_LOOP (28)
#define TEST_MODE_BALL_GROOVE_ZERO   (29)
#define TEST_MODE_LINE_FOLLOW_BALL_ACCEL_OPEN_LOOP (30)
#define TEST_MODE_BALL_VISION_OSCILLATION (31)
#define TEST_MODE_OLED_TASK_1       (32)
#define TEST_MODE_LINE_FOLLOW_TASK_2 (33)

#define TEST_MODE                    TEST_MODE_ENCODER


#endif
