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
// 使用逐飞科技调试下载器连接
//      直接将下载器正确连接在核心板的调试下载接口即可
//
// 接入红孩儿 CCD 模块
//      模块管脚            单片机管脚
//      CLK                 查看 zf_device_tsl1401.h 中 TSL1401_CLK_PIN 	宏定义
//      SI                  查看 zf_device_tsl1401.h 中 TSL1401_SI_PIN 		宏定义
//      AO[x]               查看 zf_device_tsl1401.h 中 TSL1401_AO_PIN_LIST 宏定义
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源

// *************************** 例程使用步骤说明 ***************************
// 1.根据硬件连接说明连接好模块，使用电源供电(下载器供电会导致模块电压不足)
//
// 2.下载例程到单片机中，打开逐飞助手上位机。
//
// 3.在逐飞助手上位机中，选择图像传输。
//
// 4.选择下载器对应的串口号，波特率(默认115200)，点击连接
//
// 5.等待几秒钟，图像就会显示在逐飞串口助手中。


// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程，单独使用核心板与调试下载器或者 USB-TTL 模块，并连接好红孩儿 CCD 模块，在断电情况下完成连接
// 2.将调试下载器或者 USB-TTL 模块连接电脑，完成上电
// 3.电脑上使用逐飞助手打开对应的串口，串口波特率为 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义 默认 115200，核心板按下复位按键
// 4.可以在逐飞助手的图像传输界面看到对应波形
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************

seekfree_assistant_ccd_struct seekfree_assistant_ccd1;
seekfree_assistant_ccd_struct seekfree_assistant_ccd2;
int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>
    debug_init();                   // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
    
    // 初始化CCD
    tsl1401_init(0);
    tsl1401_init(1);
    // 设置逐飞助手驱动通过串口传输
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    
    // 配置CCD图像的发送配置
    seekfree_assistant_ccd_config(&seekfree_assistant_ccd1, SEEKFREE_ASSISTANT_DATA_TYPE_UINT16, 1, RGB565_RED, TSL1401_DATA_LEN, &tsl1401_data[0][0]);
    seekfree_assistant_ccd_config(&seekfree_assistant_ccd2, SEEKFREE_ASSISTANT_DATA_TYPE_UINT16, 2, RGB565_GREEN, TSL1401_DATA_LEN, &tsl1401_data[1][0]);
    
    
    // 此处编写用户代码 例如外设初始化代码等
    
    while(true)
    {
        // 此处编写需要循环执行的代码

        if(tsl1401_finish_flag)
        {
            tsl1401_finish_flag = 0;
            seekfree_assistant_ccd_send(&seekfree_assistant_ccd1);
            seekfree_assistant_ccd_send(&seekfree_assistant_ccd2);
        }  
        // 此处编写需要循环执行的代码
    }
}



