#include "main.h"  // IWYU pragma: export
#include "adc.h"
#include "can.h"
#include "ccp_interface.h"
#include "delay.h"
#include "foc_interface.h"
#include "gd32f30x.h"
#include "gpio.h"
#include "position_sensor.h"
#include "systick.h"
#include "tim.h"
#include "usart.h"


volatile uint32_t DWT_Count = 0;

bool pin = false;

void DWT_Init(void);
void Temperature_Protect(void);
void daq_trigger(void);
void nvic_config(void);
void EXIT_Config(void);

void relay_init(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
  systick_config();  // systick provides delay_ms
  TIM1_Init();       // TIM1 provides delay_us
  DWT_Init();
  /* initialize Serial port */
  //< For USART DMA, USART must be initialized before DMA >//
  USART_Init(&husart0);
  USART_DMA_Init();
  /* initialize GPIO */
  GPIO_Init(GPIOD, &GPIOD_InitStruct);
  GPIO_Init(GPIOB, &GPIOB_InitStruct);
  GPIO_Init(GPIOE, &GPIOE_InitStruct);
  /* initialize Position_Sensor */
  Position_Sensor_Init();
  /* initialize Timer */
  TIM0_PWM_Init();
  /* initialize external interrupt */
  EXIT_Config();
  /* initialize ADC */
  //< For ADC DMA, DMA must be initialized before ADC >//
  ADC_DMA_Init();
  ADC_Init();
  /* initialize CAN and CCP */
  CAN_Init();
  ccpInit();
  /* open fan and relay */
  relay_init();
  /* configure NVIC and enable interrupt */
  nvic_config();
  while (1)
  {
    can_rx_message_t can_msg;
    if (Interface_ReadCANMessage(&can_msg))
    {
      if (can_msg.Protocol == CCP)
      {
        ccpCommand(can_msg.msg.ccp_msg.data);
      }
    }
    Interface_CANTXProcess();

    daq_trigger();
    ccpSendCallBack();
    Interface_GateState();

    pin = gpio_input_bit_get(GPIOE, GPIO_PIN_15);
    // DWT_Count = DWT->CYCCNT; // 读取DWT计数器
    Temperature_Protect();
  }
}

/*!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void nvic_config(void)
{
  nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);  // 设置中断优先级分组
  nvic_irq_enable(TIMER0_BRK_IRQn, 0, 0);
  nvic_irq_enable(EXTI5_9_IRQn, 1U, 0U);
  nvic_irq_enable(ADC0_1_IRQn, 2, 0);
  nvic_irq_enable(TIMER3_IRQn, 3, 0);
  nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 5, 0);
  /* SysTick_IRQn 009U */

  adc_interrupt_enable(ADC0, ADC_INT_EOIC);
}

void daq_trigger(void)
{
  static uint32_t last_daq_ms = 0;
  if ((systick_ms - last_daq_ms) >= 5)
  {
    last_daq_ms = systick_ms;
    ccpDaq(0);
  }
}

void relay_init(void)
{
  gpio_bit_set(SOFT_OPEN_PORT, SOFT_OPEN_PIN);
  // gpio_bit_set(FAN_OPEN_PORT, FAN_OPEN_PIN);
}

void EXIT_Config(void)
{
  gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_7);
  exti_init(EXTI_7, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
  exti_interrupt_flag_clear(EXTI_7);
}

void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // 使能DWT模块
  DWT->CYCCNT = 0;                                 // 清零
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // 启用CYCCNT
}

/* Since Temperature is not used in User code, it is treated as system function */
/* reach up interface layer to get control parameter and update protect flag */
void Temperature_Protect(void)
{
  if (Temperature > 0.35 * Protect.Temperature)
  {
    gpio_bit_set(FAN_OPEN_PORT, FAN_OPEN_PIN);
  }
  if (Temperature > Protect.Temperature)
  {
    STOP = 1;
    Protect.Flag |= Over_Heat;
  }
}
