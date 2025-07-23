#ifndef _GPIO_H
#define _GPIO_H

#include "gd32f30x_gpio.h"

#define SOFT_OPEN_PORT GPIOD
#define SOFT_OPEN_PIN GPIO_PIN_9
#define FAN_OPEN_PORT GPIOD
#define FAN_OPEN_PIN GPIO_PIN_8


void GPIO_Init(void);

#endif /* GD_GPIO_H */
