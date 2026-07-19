/*********************************************************************************************************************
* MSPM0G3507 Opensource Library 即（MSPM0G3507 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MSPM0G3507 开源库的一部分
* 
* MSPM0G3507 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          mian
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK 5.37
* 适用平台          MSPM0G3507
* 店铺链接          https://seekfree.taobao.com/
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "test_config.h"

#if (TEST_MODE == TEST_MODE_ILI9341)
#include "test_ili9341.h"
#elif (TEST_MODE == TEST_MODE_XPT2046)
#include "test_xpt2046.h"
#elif (TEST_MODE == TEST_MODE_MOTOR)
#include "test_motor.h"
#elif (TEST_MODE == TEST_MODE_ENCODER)
#include "test_encoder.h"
#elif (TEST_MODE == TEST_MODE_SPEED_PID)
#include "test_speed_pid.h"
#elif (TEST_MODE == TEST_MODE_IMU_UART)
#include "test_imu_uart.h"
#elif (TEST_MODE == TEST_MODE_ODOMETRY)
#include "test_odometry.h"
#elif (TEST_MODE == TEST_MODE_GRAY_SENSOR)
#include "test_gray_sensor.h"
#elif (TEST_MODE == TEST_MODE_VOFA_SPEED)
#include "test_vofa_speed.h"
#elif (TEST_MODE == TEST_MODE_LINE_TRACKER)
#include "test_line_tracker.h"
#elif (TEST_MODE == TEST_MODE_GPIO_OUTPUT)
#include "test_gpio_output.h"
#elif (TEST_MODE == TEST_MODE_GPIO_INPUT)
#include "test_gpio_input.h"
#elif (TEST_MODE == TEST_MODE_CONTROL_SCHEDULER)
#include "test_control_scheduler.h"
#elif (TEST_MODE == TEST_MODE_SERVO)
#include "test_servo.h"
#elif (TEST_MODE == TEST_MODE_CHASSIS_MOTION)
#include "test_chassis_motion.h"
#elif (TEST_MODE == TEST_MODE_LINE_FOLLOW_REAL)
#include "test_line_follow_real.h"
#elif (TEST_MODE == TEST_MODE_STEPPER)
#include "test_stepper.h"
#endif
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>
    debug_init();    // 调试串口信息初始化

#if (TEST_MODE == TEST_MODE_ILI9341)
    test_ili9341_run();
#elif (TEST_MODE == TEST_MODE_XPT2046)
    test_xpt2046_run();
#elif (TEST_MODE == TEST_MODE_MOTOR)
    test_motor_run();
#elif (TEST_MODE == TEST_MODE_ENCODER)
    test_encoder_run();
#elif (TEST_MODE == TEST_MODE_SPEED_PID)
    test_speed_pid_run();
#elif (TEST_MODE == TEST_MODE_IMU_UART)
    test_imu_uart_run();
#elif (TEST_MODE == TEST_MODE_ODOMETRY)
    test_odometry_run();
#elif (TEST_MODE == TEST_MODE_GRAY_SENSOR)
    test_gray_sensor_run();
#elif (TEST_MODE == TEST_MODE_VOFA_SPEED)
    test_vofa_speed_run();
#elif (TEST_MODE == TEST_MODE_LINE_TRACKER)
    test_line_tracker_run();
#elif (TEST_MODE == TEST_MODE_GPIO_OUTPUT)
    test_gpio_output_run();
#elif (TEST_MODE == TEST_MODE_GPIO_INPUT)
    test_gpio_input_run();
#elif (TEST_MODE == TEST_MODE_CONTROL_SCHEDULER)
    test_control_scheduler_run();
#elif (TEST_MODE == TEST_MODE_SERVO)
    test_servo_run();
#elif (TEST_MODE == TEST_MODE_CHASSIS_MOTION)
    test_chassis_motion_run();
#elif (TEST_MODE == TEST_MODE_LINE_FOLLOW_REAL)
    test_line_follow_real_run();
#elif (TEST_MODE == TEST_MODE_STEPPER)
    test_stepper_run();
#endif

    while (true)
    {
    }
}
