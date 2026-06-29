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

// *************************** 例程硬件连接说明 ***************************
// 使用逐飞科技调试下载器连接
//      直接将下载器正确连接在核心板的调试下载接口即可
//
// 接入 GS08RA 模块
//			模块引脚		单片机引脚
//			S0              查看 zf_device_gs08ra.h 中的 GS08RA_S0_PIN
//			S1              查看 zf_device_gs08ra.h 中的 GS08RA_S1_PIN
//			S2              查看 zf_device_gs08ra.h 中的 GS08RA_S2_PIN
//			S3              不需要连接，悬空即可
//			OUT             查看 zf_device_gs08ra.h 中的 GS08RA_OUT_PIN
//			3V3             3.3V电源
//			GND             电源地

// *************************** 例程使用步骤说明 ***************************
// 1.根据硬件连接说明连接好模块，使用电源供电(下载器供电会导致模块电压不足)
//
// 2.下载例程到单片机中，打开逐飞助手上位机。
//
// 3.在逐飞助手上位机中，选择串口传输。
//
// 4.选择下载器对应的串口号，波特率(默认115200)，点击连接
//
// 5.等待几秒钟，数据就会显示在逐飞串口助手中。

// *************************** 例程功能说明 ***************************
// 1.例程调试功能需要搭配使用主板的按键模块进行功能触发。
//
// 2.按键1的功能为：重置临时记录的光电管最大最小值
//
// 3.按键2的功能为：写入临时记录的光电管最大最小值
//
// 4.按键3的功能为：增大光电管二值化使用的阈值
//
// 5.按键4的功能为：减小光电管二值化使用的阈值

// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程，单独使用核心板与调试下载器或者 USB-TTL 模块，并连接好GS08RA模块，在断电情况下完成连接
//
// 2.将调试下载器或者 USB-TTL 模块连接电脑，完成上电
//
// 3.电脑上使用逐飞助手打开对应的串口，串口波特率为 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义 默认 115200，核心板按下复位按键
//
// 4.可以在逐飞助手的串口助手界面看到对应数据，也可以在示波器界面使用printf模式看到数据的波形图
//
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************
// 光电管 最大最小值采集
uint16 gray_max[8] = {0};
uint16 gray_min[8] = {4095,4095,4095,4095,4095,4095,4095,4095};  // 12位ADC默认最大
uint8  gray_threshold = 30;	  // 灰度阈值

// 采集并更新光电管最大最小值
void gray_max_min_update(void)
{
    uint8 i;
    for(i=0; i<8; i++)
    {
        if(gs08ra_raw_val[i] > gray_max[i]) gray_max[i] = gs08ra_raw_val[i];
        if(gs08ra_raw_val[i] < gray_min[i]) gray_min[i] = gs08ra_raw_val[i];
    }
}

// 重置最大最小值
void gray_max_min_reset(void)
{
    uint8 i;
    for(i=0; i<8; i++)
    {
        gray_max[i] = 0;
        gray_min[i] = 4095;
    }
}
// 将采集的最大最小值直接写入 gs08ra_max_val / gs08ra_min_val
void gray_save_max_min_to_array(void)
{
    uint8 i;
    for(i=0; i<8; i++)
    {
        gs08ra_max_val[i] = gray_max[i];  // 写入最大值数组
        gs08ra_min_val[i] = gray_min[i];  // 写入最小值数组
    }
}


int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>
    debug_init();					// 调试串口信息初始化
	// 此处编写用户代码 例如外设初始化代码等
	static int16 cnt = 0;
	// 初始化光电管
	gs08ra_init();
	
	// 按键初始化
	key_init(10);
	// 此处编写用户代码 例如外设初始化代码等

    while(true)
    {
		// 此处编写需要循环执行的代码
		cnt++;
		gs08ra_scan_read();		// 获取灰度传感器的值
		gray_max_min_update();	// 更新灰度传感器最大最小值
		if(cnt % 10 == 0)
		{
			printf("RAW:%d,%d,%d,%d,%d,%d,%d,%d\r\n",gs08ra_raw_val[0],gs08ra_raw_val[1],gs08ra_raw_val[2],gs08ra_raw_val[3],gs08ra_raw_val[4],gs08ra_raw_val[5],gs08ra_raw_val[6],gs08ra_raw_val[7]);
			printf("MAX:%d,%d,%d,%d,%d,%d,%d,%d\r\n", gray_max[0],gray_max[1],gray_max[2],gray_max[3],gray_max[4],gray_max[5],gray_max[6],gray_max[7]);
			printf("MIN:%d,%d,%d,%d,%d,%d,%d,%d\r\n", gray_min[0],gray_min[1],gray_min[2],gray_min[3],gray_min[4],gray_min[5],gray_min[6],gray_min[7]);
			printf("DEAL:%d,%d,%d,%d,%d,%d,%d,%d\r\n",gs08ra_deal_val[0],gs08ra_deal_val[1],gs08ra_deal_val[2],gs08ra_deal_val[3],gs08ra_deal_val[4],gs08ra_deal_val[5],gs08ra_deal_val[6],gs08ra_deal_val[7]);
			printf("BIN:%d,%d,%d,%d,%d,%d,%d,%d\r\n",gs08ra_bin_val[0],gs08ra_bin_val[1],gs08ra_bin_val[2],gs08ra_bin_val[3],gs08ra_bin_val[4],gs08ra_bin_val[5],gs08ra_bin_val[6],gs08ra_bin_val[7]);
			printf("Threshold:%d\r\n", gray_threshold);
		}
		// -------------------- 按键处理 --------------------
		key_scanner();
		
		// 按键1：切换显示 最大/最小值
		if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
		{
			gray_max_min_reset();
			printf("//=====================//\r\n");
			printf("Reset Max and Min\n");
			printf("//=====================//\r\n");
			system_delay_ms(1000);
		}
		// 按键2：设置最大最小值进入
		if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
		{
			gray_save_max_min_to_array();
			printf("//=====================//\r\n");
			printf("Record Max and Min\n");
			printf("//=====================//\r\n");
			system_delay_ms(1000);
		}
		// 按键3：增大光电管阈值
		if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
		{
			gray_threshold += 5;
			gs08ra_set_threshold(gray_threshold);
		}
		// 按键4：减小光电管阈值
		if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
		{
			gray_threshold -= 4;
			gs08ra_set_threshold(gray_threshold);
		}
		
		system_delay_ms(5);
		
        // 此处编写需要循环执行的代码
    }
}

