#include "tim0.h"

float PWM_ARR;

static inline uint8_t calculate_deadtime_value(uint32_t deadtime_ns, uint32_t timer_clk_hz);

void TIM0_PWM_Init(void)
{

    // 1. 时钟与复用
    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_AF);
    gpio_pin_remap_config(GPIO_TIMER0_FULL_REMAP, ENABLE);

    // 2. PE8~PE13 为复用推挽输出
    gpio_init(GPIOE, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
              GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
                  GPIO_PIN_12 | GPIO_PIN_13);

    // 3. 定时器基本参数
    timer_deinit(TIMER0);
    timer_parameter_struct timer_initpara;
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler = 1;
    timer_initpara.alignedmode = TIMER_COUNTER_CENTER_BOTH;
    timer_initpara.counterdirection = TIMER_COUNTER_UP; // no sense
    timer_initpara.period = 3000 - 1;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 1;// every 2 update events calls 1 interrupt
    timer_init(TIMER0, &timer_initpara);

    // 可选 ARR shadow register
    timer_auto_reload_shadow_enable(TIMER0);
    

    // 4. 通道配置
    timer_oc_parameter_struct oc_param;
    oc_param.outputstate = TIMER_CCX_ENABLE;
    oc_param.outputnstate = TIMER_CCXN_ENABLE;
    oc_param.ocpolarity = TIMER_OC_POLARITY_LOW;
    oc_param.ocnpolarity = TIMER_OCN_POLARITY_LOW;
    oc_param.ocidlestate = TIMER_OC_IDLE_STATE_HIGH;
    oc_param.ocnidlestate = TIMER_OCN_IDLE_STATE_HIGH;

    for (int ch = TIMER_CH_0; ch <= TIMER_CH_2; ch++)
    {
        timer_channel_output_config(TIMER0, ch, &oc_param);
        timer_channel_output_mode_config(TIMER0, ch, TIMER_OC_MODE_PWM1);//PWM1: ccr > arr output high
        timer_channel_output_shadow_config(TIMER0, ch, TIMER_OC_SHADOW_ENABLE);
        timer_channel_output_pulse_value_config(TIMER0, ch, 0);
    }
    timer_oc_parameter_struct oc_param_ch3;
    oc_param_ch3.outputstate = TIMER_CCX_DISABLE;       // no output no input
    oc_param_ch3.outputnstate = TIMER_CCXN_DISABLE;
    oc_param_ch3.ocpolarity = TIMER_OC_POLARITY_HIGH;   // no sense
    oc_param_ch3.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    oc_param_ch3.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    oc_param_ch3.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    
    timer_channel_output_config(TIMER0, TIMER_CH_3, &oc_param_ch3);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_3, TIMER_OC_MODE_TIMING); // compare only
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_3, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3, 3000 - 1); // 

    // 5. 死区 + BRK
    timer_break_parameter_struct brk_param;
    brk_param.runoffstate = TIMER_ROS_STATE_ENABLE;
    brk_param.ideloffstate = TIMER_IOS_STATE_ENABLE;
    brk_param.deadtime = calculate_deadtime_value(2000, SystemCoreClock); // 2us
    brk_param.breakstate = TIMER_BREAK_DISABLE;
    brk_param.breakpolarity = TIMER_BREAK_POLARITY_LOW;
    brk_param.protectmode = TIMER_CCHP_PROT_OFF;
    brk_param.outputautostate = TIMER_OUTAUTO_ENABLE;
    timer_break_config(TIMER0, &brk_param);

    // 6. 中断配置
    timer_interrupt_enable(TIMER0, TIMER_INT_CH3);
    
    // 7. 主输出使能 + 启动
    timer_primary_output_config(TIMER0, ENABLE);
    timer_enable(TIMER0);
    PWM_ARR = (float)(TIMER_CAR(TIMER0) + 1);
}

static inline uint8_t calculate_deadtime_value(uint32_t deadtime_ns, uint32_t timer_clk_hz)
{
    float t_dts = 1e9f / timer_clk_hz; // ns
    float ticks = deadtime_ns / t_dts;

    if (ticks <= 127)
        return (uint8_t)(ticks);
    else if (ticks <= (64 + 63) * 2)
        return (uint8_t)(0x80 | ((uint8_t)((ticks / 2) - 64) & 0x3F));
    else if (ticks <= (32 + 31) * 8)
        return (uint8_t)(0xC0 | ((uint8_t)((ticks / 8) - 32) & 0x1F));
    else if (ticks <= (32 + 31) * 16)
        return (uint8_t)(0xE0 | ((uint8_t)((ticks / 16) - 32) & 0x1F));
    else
        return 0xFF; // Max
}
