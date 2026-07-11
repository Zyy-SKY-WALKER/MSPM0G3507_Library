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
#include "ml_oled.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

static const uint8_t oled_test_bitmap[32] =
{
    0xFFU, 0x01U, 0x01U, 0x01U, 0x01U, 0x01U, 0x01U, 0x01U,
    0x01U, 0x01U, 0x01U, 0x01U, 0x01U, 0x01U, 0x01U, 0xFFU,
    0xFFU, 0x80U, 0x80U, 0x80U, 0x80U, 0x80U, 0x80U, 0x80U,
    0x80U, 0x80U, 0x80U, 0x80U, 0x80U, 0x80U, 0x80U, 0xFFU
};

/**
 * @brief       Report an OLED test failure through the debug UART.
 * @param       result Test function return value.
 * @param       test_name Test item name.
 * @retval      None.
 */
static void oled_test_report(bool result, const char *test_name)
{
    if (!result)
    {
        printf("OLED %s test failed.\r\n", test_name);
    }
}

/**
 * @brief       Keep the current test page for two seconds and clear it.
 * @retval      None.
 */
static void oled_test_wait_and_clear(void)
{
    system_delay_ms(2000U);
    oled_test_report(ml_oled_clear(), "clear");
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>
    debug_init();    // 调试串口信息初始化
//    uart_init (UART_1,115200,UART1_TX_B6,UART1_RX_B7) ;
//    uart_write_byte (UART_1,1);
//    uart_write_byte (UART_1,0);
//    uart_write_byte (UART_1,2);
//    uart_write_byte (UART_1,4);

    // 此处编写用户代码 例如外设初始化代码等
    gpio_init(A14, GPO, GPIO_LOW, GPO_PUSH_PULL);

    if (!ml_oled_init())
    {
        printf("ml_oled_init failed.\r\n");

        while (true)
        {
        }
    }

    while (true)
    {
        oled_test_report(ml_oled_show_string(1U, 1U, "CHAR"),
                         "character title");
        oled_test_report(ml_oled_show_char(2U, 1U, 'A'), "character");
        oled_test_wait_and_clear();

        oled_test_report(ml_oled_show_string(1U, 1U, "STRING"),
                         "string title");
        oled_test_report(ml_oled_show_string(2U, 1U, "MSPM0G3507"),
                         "string");
        oled_test_wait_and_clear();

        oled_test_report(ml_oled_show_string(1U, 1U, "UINT"),
                         "unsigned title");
        oled_test_report(ml_oled_show_uint(2U, 1U, 12345U, 5U),
                         "unsigned integer");
        oled_test_wait_and_clear();

        oled_test_report(ml_oled_show_string(1U, 1U, "SIGNED INT"),
                         "signed title");
        oled_test_report(ml_oled_show_int(2U, 1U, 42, 4U),
                         "positive integer");
        oled_test_report(ml_oled_show_int(3U, 1U, -42, 4U),
                         "negative integer");
        oled_test_wait_and_clear();

        oled_test_report(ml_oled_show_string(1U, 1U, "HEX"),
                         "hex title");
        oled_test_report(ml_oled_show_hex(2U, 1U, 0x12ABCD78U, 8U),
                         "hex");
        oled_test_wait_and_clear();

        oled_test_report(ml_oled_show_string(1U, 1U, "BINARY"),
                         "binary title");
        oled_test_report(ml_oled_show_binary(2U, 1U, 0xA5U, 8U),
                         "binary");
        oled_test_wait_and_clear();

        oled_test_report(ml_oled_show_string(1U, 1U, "FLOAT"),
                         "float title");
        oled_test_report(ml_oled_show_float(2U, 1U, 3.14F, 2U, 2U),
                         "positive float");
        oled_test_report(ml_oled_show_float(3U, 1U, -12.345F, 2U, 3U),
                         "negative float");
        oled_test_wait_and_clear();

        oled_test_report(ml_oled_show_string(1U, 1U, "BITMAP"),
                         "bitmap title");
        oled_test_report(ml_oled_draw_bitmap(56U, 3U, 72U, 5U,
                                              oled_test_bitmap),
                         "bitmap");
        oled_test_wait_and_clear();
    }
}

