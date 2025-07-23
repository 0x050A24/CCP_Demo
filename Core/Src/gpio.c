#include "gpio.h"

typedef struct
{
    uint32_t Pin;       /*!< 指定要配置的GPIO管脚，可使用 GPIO_PIN_x 宏的组合 */
    uint32_t Mode;      /*!< 指定选中管脚的工作模式，例如 GPIO_MODE_OUT_PP、GPIO_MODE_IN_FLOATING、GPIO_MODE_AF_PP 等 */
    uint32_t Speed;     /*!< 指定选中管脚的输出速率，例如 GPIO_OSPEED_50MHZ */
    uint32_t Alternate; /*!< 指定选中管脚的复用功能（仅当模式为复用时有效） */
} GPIO_InitTypeDef;

static inline void GPIO_Config(uint32_t GPIOx, GPIO_InitTypeDef *GPIO_InitStruct);

void GPIO_Init(void)
{

    GPIO_InitTypeDef GPIOD_InitStruct = {
        .Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
        .Mode = GPIO_MODE_OUT_PP,
        .Speed = GPIO_OSPEED_2MHZ,
        .Alternate = 0
    };
    
    GPIO_InitTypeDef GPIOB_InitStruct = {
        .Pin = GPIO_PIN_7,
        .Mode = GPIO_MODE_IPU,
        .Speed = GPIO_OSPEED_50MHZ,
        .Alternate = 0
    };

    GPIO_InitTypeDef GPIOE_InitStruct = {
        .Pin = GPIO_PIN_7,
        .Mode = GPIO_MODE_IN_FLOATING,
        .Speed = GPIO_OSPEED_50MHZ,
        .Alternate = 0
    };

    // 调用GPIO初始化函数
    GPIO_Config(GPIOD, &GPIOD_InitStruct);
    GPIO_Config(GPIOB, &GPIOB_InitStruct);
    GPIO_Config(GPIOE, &GPIOE_InitStruct);
}

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

static inline void GPIO_Config(uint32_t GPIOx, GPIO_InitTypeDef *GPIO_InitStruct)
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
