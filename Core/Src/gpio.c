#include "gpio.h"

/**
 * @brief  根据GPIOx的地址使能对应GPIO时钟
 */
GPIO_InitTypeDef GPIOD_InitStruct = {
    .Pin = GPIO_PIN_7 |GPIO_PIN_8 | GPIO_PIN_9,
    .Mode = GPIO_MODE_OUT_PP,  // 输出推挽模式
    .Speed = GPIO_OSPEED_2MHZ, // 输出速率2MHz
    .Alternate = 0             // 复用功能未使用
};

GPIO_InitTypeDef GPIOB_InitStruct = {
    .Pin = GPIO_PIN_7,
    .Mode = GPIO_MODE_IPU,  // 上拉输入模式
    .Speed = GPIO_OSPEED_50MHZ, // 
    .Alternate = 0             // 复用功能未使用
};

static void GPIO_Clock_Enable(uint32_t GPIOx)
{
    if (GPIOx == GPIOA)
    {
        rcu_periph_clock_enable(RCU_GPIOA);
    }
    else if (GPIOx == GPIOB)
    {
        rcu_periph_clock_enable(RCU_GPIOB);
    }
    else if (GPIOx == GPIOC)
    {
        rcu_periph_clock_enable(RCU_GPIOC);
    }
    else if (GPIOx == GPIOD)
    {
        rcu_periph_clock_enable(RCU_GPIOD);
    }
    // 如果有其它GPIO端口，根据需要扩展
}

void GPIO_Init(uint32_t GPIOx, GPIO_InitTypeDef *GPIO_InitStruct)
{
    // 自动使能GPIO时钟
    GPIO_Clock_Enable(GPIOx);
    if ((GPIO_InitStruct->Mode == GPIO_MODE_AF_PP || GPIO_InitStruct->Mode == GPIO_MODE_AF_OD) &&
        (GPIO_InitStruct->Alternate != 0))
    {
        gpio_pin_remap_config(GPIO_InitStruct->Alternate, ENABLE);
    }
    gpio_init(GPIOx, GPIO_InitStruct->Mode, GPIO_InitStruct->Speed, GPIO_InitStruct->Pin);
}
