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
* 
* 修改记录
* 日期              作者                备注
* 2025-06-1        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完


// *************************** 例程硬件连接说明 ***************************
// 使用逐飞科技 CMSIS-DAP | ARM 调试下载器连接
//      直接将下载器正确连接在核心板的调试下载接口即可
// 
// 使用 USB-TTL 模块连接
//      模块管脚            单片机管脚
//      USB-TTL-RX          查看 zf_common_debug.h 文件中 DEBUG_UART_TX_PIN 宏定义的引脚 默认 A10
//      USB-TTL-TX          查看 zf_common_debug.h 文件中 DEBUG_UART_RX_PIN 宏定义的引脚 默认 A11
//      USB-TTL-GND         核心板电源地 GND
//      USB-TTL-3V3         核心板 3V3 电源
// 
// 绝对值编码器（角度传感器）接线
//      模块管脚            单片机管脚
//      SCLK                查看 zf_device_absolute_encoder.h 中 ABSOLUTE_ENCODER_SCLK_PIN 宏定义
//      MOSI                查看 zf_device_absolute_encoder.h 中 ABSOLUTE_ENCODER_MOSI_PIN 宏定义
//      MISO                查看 zf_device_absolute_encoder.h 中 ABSOLUTE_ENCODER_MISO_PIN 宏定义
//      CS                  查看 zf_device_absolute_encoder.h 中 ABSOLUTE_ENCODER_CS_PIN 宏定义 4个编码器分别使用不同的CS
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源



// *************************** 例程测试说明 ***************************
// 1.核心板烧录本例程 插在主板上 绝对值编码器按照上述硬件连接方式接好
// 
// 2.电池供电 上电后会从串口输出编码器采集的位置信息 如下
//      location = 	xxxx,xxxx,xxxx,xxxx
//      offset =  	xxxx,xxxx,xxxx,xxxx
// 
// 3.转动编码器数据会有变化
// 
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************


volatile uint8_t pit_flag    = 0;

uint8 encoder_init_data = 0;

int16 location_data1 = 0 , location_data2 =0 ,location_data3 =0 ,location_data4 =0;
int16 offset_data1 = 0,offset_data2 = 0,offset_data3 = 0,offset_data4 = 0;


void absolute_encoder_pit_handler (uint32 event, void *ptr)
{
	 *((uint8 *)ptr)  = 1;
	
	 location_data1 =  absolute_encoder_get_location(0);		//获取编码器当前大角度信息
		 offset_data1 =  absolute_encoder_get_offset(0);			//通过两次角度对比得到当前的旋转速度
	
	 location_data2 =  absolute_encoder_get_location(1);		//获取编码器当前大角度信息
		 offset_data2 =  absolute_encoder_get_offset(1);      //通过两次角度对比得到当前的旋转速度
	
	 location_data3 =  absolute_encoder_get_location(2);		//获取编码器当前大角度信息
		 offset_data3 =  absolute_encoder_get_offset(2);      //通过两次角度对比得到当前的旋转速度
	
	 location_data4 =  absolute_encoder_get_location(3);		//获取编码器当前大角度信息
		 offset_data4 =  absolute_encoder_get_offset(3);      //通过两次角度对比得到当前的旋转速度
	
}


int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>
    debug_init();										// 调试串口信息初始化
		// 此处编写用户代码 例如外设初始化代码等

		for(encoder_init_data = 0;encoder_init_data < 4; encoder_init_data ++)
		{
			if(absolute_encoder_init(encoder_init_data))					//初始化4个编码器
			{

				printf("encoder %d fail\r\n",encoder_init_data+1);	//提示编码器初始化失败
			}
			else 
			{
				printf("encoder %d successfully\r\n",encoder_init_data+1);		//提示编码器初始化成功
			}
			system_delay_ms(500);
		}

    pit_ms_init(PIT_TIM_G12, 20, absolute_encoder_pit_handler, (void *)&pit_flag);


    while(true)
    {
        // 此处编写需要循环执行的代码

			if(pit_flag)																				//在串口上位机上显示角度和转速信息
        {
						printf("location =  %d,%d,%d,%d\r\n",location_data1,location_data2,location_data3,location_data4);
						printf("offset =  %d,%d,%d,%d\r\n",offset_data1,offset_data2,offset_data3,offset_data4);
            pit_flag = 0;
        }
        // 此处编写需要循环执行的代码
    }
}



