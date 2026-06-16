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
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

// MSPM0G3507单片机 仅仅只有一个QEI单元，因此仅支持一个正交编码器
#define ENCODER1_TIMER  TIM_G8                  // 定义编码器方向引脚
#define ENCODER1_A      TIMG8_ENCODER1_CH1_A26  // 定义编码器脉冲引脚    
#define ENCODER1_B      TIMG8_ENCODER1_CH2_A27  // 定义编码器方向引脚


#define PIT_TIMER       ( PIT_TIM_A0 )          // 定义周期中断用的定时器

int16 encoder;                                  // 编码器数据    


void pit_callback(uint32 event, void *ptr)
{
    encoder = encoder_get_count(ENCODER1_TIMER);    // 采集编码器数据       
    encoder_clear_count(ENCODER1_TIMER);            // 编码器数据采集完成后务必清零 

}

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>
    debug_init();					// 调试串口信息初始化
	// 此处编写用户代码 例如外设初始化代码等

    encoder_quad_init(ENCODER1_TIMER, ENCODER1_A, ENCODER1_B);  // 初始化编码器1端口  
    pit_ms_init(PIT_TIMER, 100, pit_callback, NULL);
    // 此处编写用户代码 例如外设初始化代码等

    while(true)
    {
        printf("QUAD ENCODER counter \t%d .\r\n", encoder);        // 输出编码器计数信息

        system_delay_ms(100);
    }
}

