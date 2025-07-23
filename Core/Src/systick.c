#include "gd32f30x.h"
#include "systick.h"

volatile uint32_t systick_ms = 0;
static volatile uint32_t delay;
uint32_t AHB_CLK;
uint32_t APB1_CLK;
uint32_t APB2_CLK;

/*!
    \brief      configure systick
    \param[in]  none
    \param[out] none
    \retval     none
*/
void systick_config(void)
{
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / 1000U)){
        /* capture error */
        while (1){
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x09U);
    AHB_CLK = rcu_clock_freq_get(CK_AHB);
    APB1_CLK = rcu_clock_freq_get(CK_APB1);
    APB2_CLK = rcu_clock_freq_get(CK_APB2);
}

/*!
    \brief      delay a time in milliseconds
    \param[in]  count: count in milliseconds
    \param[out] none
    \retval     none
*/
void delay_ms(uint32_t count)
{
    delay = count;

    while(0U != delay){
    }
}

/*!
    \brief      delay decrement
    \param[in]  none
    \param[out] none
    \retval     none
*/
void delay_decrement(void)
{
    if (0U != delay){
        delay--;
    }
}
/*!
    \brief      delay a time in microseconds
    \param[in]  count: count in microseconds
    \param[out] none
    \retval     none
*/


