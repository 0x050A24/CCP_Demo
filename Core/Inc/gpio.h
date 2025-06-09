#ifndef _GPIO_H
#define _GPIO_H

#include "gd32f30x.h"

#define SOFT_OPEN_PORT GPIOD
#define SOFT_OPEN_PIN GPIO_PIN_9
#define FAN_OPEN_PORT GPIOD
#define FAN_OPEN_PIN GPIO_PIN_8

/* GPIO初始化句柄结构体 */
/* GPIO配置结构体 */

/*param[in]  mode: gpio pin mode
                only one parameter can be selected which is shown as below:
      \arg        GPIO_MODE_AIN: analog input mode
      \arg        GPIO_MODE_IN_FLOATING: floating input mode
      \arg        GPIO_MODE_IPD: pull-down input mode
      \arg        GPIO_MODE_IPU: pull-up input mode
      \arg        GPIO_MODE_OUT_OD: GPIO output with open-drain
      \arg        GPIO_MODE_OUT_PP: GPIO output with push-pull
      \arg        GPIO_MODE_AF_OD: AFIO output with open-drain
      \arg        GPIO_MODE_AF_PP: AFIO output with push-pull
*/

typedef struct
{
    uint32_t Pin;       /*!< 指定要配置的GPIO管脚，可使用 GPIO_PIN_x 宏的组合 */
    uint32_t Mode;      /*!< 指定选中管脚的工作模式，例如 GPIO_MODE_OUT_PP、GPIO_MODE_IN_FLOATING、GPIO_MODE_AF_PP 等 */
    uint32_t Speed;     /*!< 指定选中管脚的输出速率，例如 GPIO_OSPEED_50MHZ */
    uint32_t Alternate; /*!< 指定选中管脚的复用功能（仅当模式为复用时有效） */
} GPIO_InitTypeDef;

void GPIO_Init(uint32_t GPIOx, GPIO_InitTypeDef *GPIO_InitStruct);

#endif /* GD_GPIO_H */
