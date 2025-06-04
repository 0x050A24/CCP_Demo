
#include "main.h"

#define DAQ_INTERVAL_CYCLES 600000 // 5ms * 120MHz

uint16_t test_var1 = 0;
int16_t test_var2 = 0;
float_t float_var = 0.0f;
static uint32_t dwt_start_cycle = 0;
void trigger_daq_every_5ms(void);
void nvic_config(void);
void dwt_init(void);
/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    dwt_init();
    systick_config();
    /* initialize Serial port */
    USART_Init(&husart0);
    /* configure NVIC */
    nvic_config();
    /* initialize CAN and CAN filter */
    CAN_Init(&hcan0);
    ccpInit();

    /* enable CAN receive FIFO0 not empty interrupt */
    can_interrupt_enable(hcan0.Instance, CAN_INTEN_RFNEIE0);

    while (1)
    {
        float_var += 0.1f;
        test_var2--;
        test_var1++;

        process_can_rx_buffer();

        trigger_daq_every_5ms();

        uint32_t CallBack_cycles = DWT->CYCCNT;
        ccpSendCallBack();
        uint32_t CallBack_elapsed_cycles = DWT->CYCCNT - CallBack_cycles;
        printf("\r\nCallBack_elapsed_cycles is %ld\n", CallBack_elapsed_cycles);
        printf("DWT->CYCCNT = %lu\r\n", DWT->CYCCNT);
        // printf("DWT->CTRL = 0x%08lX\n", DWT->CTRL);
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
    /* configure CAN0 NVIC */
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 1, 0);
}

void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 使能DWT模块
    DWT->CYCCNT = 0;                                // 清零
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // 启用CYCCNT

    printf("DEMCR = 0x%08lX, DWT->CTRL = 0x%08lX\n", CoreDebug->DEMCR, DWT->CTRL);
}

void trigger_daq_every_5ms(void)
{
    if ((DWT->CYCCNT - dwt_start_cycle) >= DAQ_INTERVAL_CYCLES)
    {
        dwt_start_cycle = DWT->CYCCNT;
        ccpDaq(0);
    }
}