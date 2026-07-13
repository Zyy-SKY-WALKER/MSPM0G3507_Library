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

#define TEST_MODE                    TEST_MODE_VOFA_SPEED

#endif
