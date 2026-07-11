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
* 文件名称          zf_driver_encoder
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

#include "zf_common_debug.h"
#include "zf_common_interrupt.h"

#include "zf_driver_encoder.h"

static const DL_TimerA_ClockConfig gCAPTURE_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static GPTIMER_Regs* const timer_list[] = {TIMA0, TIMA1, TIMG0, TIMG6, TIMG7, TIMG8, TIMG12};

static gpio_pin_enum encoder_dir[TIM_MAX] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ENCODER 脉冲方向模式底层初始化
// 参数说明     index           TIMER 外设模块号
// 参数说明     lsb_pin         编码器脉冲输入引脚及复用信息
// 参数说明     dir_pin         编码器方向输入引脚
// 参数说明     cc_index        定时器捕获比较通道
// 返回参数     void
// 备注信息     支持 CCP0 与 CCP1 单边沿硬件计数
//-------------------------------------------------------------------------------------------------------------------
static void encoder_dir_counter_init (timer_index_enum index, uint32 lsb_pin, gpio_pin_enum dir_pin, DL_TIMER_CC_INDEX cc_index)
{
    uint32 input_select;
    uint32 ccp_direction;
    DL_TIMER_CZC zero_control;
    DL_TIMER_CAC advance_control;
    DL_TIMER_CLC load_control;

    if(DL_TIMER_CC_0_INDEX == cc_index)
    {
        input_select = DL_TIMER_CC_IN_SEL_CCP0;
        ccp_direction = DL_TIMER_CC0_INPUT;
        zero_control = DL_TIMER_CZC_CCCTL0_ZCOND;
        advance_control = DL_TIMER_CAC_CCCTL0_ACOND;
        load_control = DL_TIMER_CLC_CCCTL0_LCOND;
    }
    else
    {
        input_select = DL_TIMER_CC_IN_SEL_CCPX;
        ccp_direction = DL_TIMER_CC1_INPUT;
        zero_control = DL_TIMER_CZC_CCCTL1_ZCOND;
        advance_control = DL_TIMER_CAC_CCCTL1_ACOND;
        load_control = DL_TIMER_CLC_CCCTL1_LCOND;
    }

    afio_init(lsb_pin & ENCODER_PIN_INDEX_MASK, GPI, (lsb_pin >> ENCODER_PIN_AF_OFFSET) & ENCODER_PIN_AF_MASK, GPI_PULL_UP);

    encoder_dir[index] = dir_pin;
    gpio_init(dir_pin, GPI, 0, GPI_PULL_UP);

    DL_Timer_setClockConfig(timer_list[index], &gCAPTURE_0ClockConfig);
    DL_Timer_setCaptureCompareInput(timer_list[index], DL_TIMER_CC_INPUT_INV_NOINVERT, input_select, cc_index);
    DL_Timer_setLoadValue(timer_list[index], 65535);
    DL_Timer_setCaptureCompareCtl(timer_list[index], DL_TIMER_CC_MODE_CAPTURE, DL_TIMER_CC_ZCOND_NONE | DL_TIMER_CC_ACOND_TRIG_RISE | DL_TIMER_CC_LCOND_NONE | DL_TIMER_CAPTURE_EDGE_DETECTION_MODE_RISING, cc_index);
    DL_Timer_setCCPDirection(timer_list[index], ccp_direction);
    DL_Timer_setCounterControl(timer_list[index], zero_control, advance_control, load_control);
    DL_Timer_setCounterMode(timer_list[index], DL_TIMER_COUNT_MODE_UP);
    DL_Timer_setCounterValueAfterEnable(timer_list[index], DL_TIMER_COUNT_AFTER_EN_NO_CHANGE);
    DL_Timer_setCounterRepeatMode(timer_list[index], DL_TIMER_REPEAT_MODE_ENABLED);
    DL_Timer_setCaptureCompareInputFilter(timer_list[index], DL_TIMER_CC_INPUT_FILT_CPV_CONSEC_PER, DL_TIMER_CC_INPUT_FILT_FP_PER_8, cc_index);
    DL_Timer_enableCaptureCompareInputFilter(timer_list[index], cc_index);
    DL_Timer_setTimerCount(timer_list[index], 0);
    DL_Timer_enableClock(timer_list[index]);
    DL_Timer_startCounter(timer_list[index]);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ENCODER 获取计数值
// 参数说明     index           TIMER 外设模块号
// 返回参数     void
// 使用示例     encoder_get_count(index);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
int16 encoder_get_count (timer_index_enum index)
{
    int16 count;
    
    count = DL_Timer_getTimerCount(timer_list[index]);
    if(0xff != encoder_dir[index])
    {
        count = gpio_get_level(encoder_dir[index]) == 1 ?  count : -count;
    }
    // else
    // {
        // // 为了与方向输出的编码器精度一致，因此这里/4，如果需要高精度可以自行去掉/4
        // count = count / 4;
    // }

    return count;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ENCODER 清除计数值
// 参数说明     index           TIMER 外设模块号
// 返回参数     void
// 使用示例     encoder_clear_count(index);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void encoder_clear_count (timer_index_enum index)
{
    DL_Timer_stopCounter(timer_list[index]);
    DL_Timer_setTimerCount(timer_list[index], 0);
    DL_Timer_startCounter(timer_list[index]);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ENCODER 正交解码模式初始化
// 参数说明     index           TIMER 外设模块号
// 参数说明     ch1_pin         通道1引脚
// 参数说明     ch2_pin         通道2引脚
// 返回参数     void
// 使用示例     encoder_quad_init(TIM_G8, TIMG8_ENCODER1_CH1_B10, TIMG8_ENCODER1_CH2_B11);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void encoder_quad_init (timer_index_enum index, encoder_channel1_enum ch1_pin, encoder_channel2_enum ch2_pin)
{
    zf_assert(TIM_G8 == index);
    zf_assert(((ch1_pin >> ENCODER_INDEX_OFFSET) & ENCODER_INDEX_MASK) == index);
    zf_assert(((ch2_pin >> ENCODER_INDEX_OFFSET) & ENCODER_INDEX_MASK) == index);
    
    afio_init(ch1_pin & ENCODER_PIN_INDEX_MASK, GPI, (ch1_pin >> ENCODER_PIN_AF_OFFSET) & ENCODER_PIN_AF_MASK, GPI_PULL_UP);
    afio_init(ch2_pin & ENCODER_PIN_INDEX_MASK, GPI, (ch2_pin >> ENCODER_PIN_AF_OFFSET) & ENCODER_PIN_AF_MASK, GPI_PULL_UP);
    
    DL_Timer_setClockConfig(timer_list[index], &gCAPTURE_0ClockConfig);

    DL_TimerG_configQEI(timer_list[index], DL_TIMER_QEI_MODE_2_INPUT, DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(timer_list[index], DL_TIMER_QEI_MODE_2_INPUT, DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);
    DL_Timer_setLoadValue(timer_list[index], 65535);
    DL_Timer_enableClock(timer_list[index]);
    DL_Timer_startCounter(timer_list[index]);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ENCODER 带方向输出模式初始化
// 参数说明     index           TIMER 外设模块号
// 参数说明     lsb_pin         脉冲输入引脚
// 参数说明     dir_pin         方向输入引脚
// 返回参数     void
// 使用示例     encoder_dir_init(TIM_G8, TIMG8_ENCODER1_CH1_B10, B11);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void encoder_dir_init (timer_index_enum index, encoder_channel1_enum lsb_pin, gpio_pin_enum dir_pin)
{
    zf_assert(((lsb_pin >> ENCODER_INDEX_OFFSET) & ENCODER_INDEX_MASK) == index);

    encoder_dir_counter_init(index, (uint32)lsb_pin, dir_pin, DL_TIMER_CC_0_INDEX);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ENCODER 脉冲方向模式初始化 使用定时器通道 2
// 参数说明     index           TIMER 外设模块号
// 参数说明     lsb_pin         编码器脉冲输入引脚
// 参数说明     dir_pin         编码器方向输入引脚
// 返回参数     void
// 使用示例     encoder_dir_timg8_ch2_init(TIMG8_ENCODER1_CH2_B22, B23);
// 备注信息     用于脉冲引脚仅支持 CCP1 的情况
//-------------------------------------------------------------------------------------------------------------------
void encoder_dir_timg8_ch2_init (encoder_channel2_enum lsb_pin, gpio_pin_enum dir_pin)
{
    zf_assert(((lsb_pin >> ENCODER_INDEX_OFFSET) & ENCODER_INDEX_MASK) == TIM_G8);

    encoder_dir_counter_init(TIM_G8, (uint32)lsb_pin, dir_pin, DL_TIMER_CC_1_INDEX);
}
