#include "can.h"
static rcu_periph_enum gpio_port_to_rcu(uint32_t port);

can_receive_message_struct can_buffer[CAN_BUFFER_SIZE];
volatile uint8_t can_buffer_head = 0;
volatile uint8_t can_buffer_tail = 0;

CAN_HandleTypeDef hcan0 = {
    .Instance = CAN0,
    .Init = {
        .TX_GPIO_Port = GPIOD,
        .TX_Pin = GPIO_PIN_1,
        .RX_GPIO_Port = GPIOD,
        .RX_Pin = GPIO_PIN_0,
        .BaudrateKbps = 1000,
    }};

/*!
    \brief      initialize CAN and Filter
    \param[in]  none
    \param[out] none
    \retval     none
*/
CAN_StatusTypeDef CAN_Init(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL || hcan->Instance == 0)
    {
        return CAN_ERROR;
    }

    const CAN_InitTypeDef *init = &hcan->Init;

    // 推断 TX/RX RCU 时钟
    rcu_periph_enum tx_gpio_rcu = gpio_port_to_rcu(init->TX_GPIO_Port);
    rcu_periph_enum rx_gpio_rcu = gpio_port_to_rcu(init->RX_GPIO_Port);

    // 根据实例动态开启 CAN 时钟
    switch (hcan->Instance)
    {
    case CAN0:
        rcu_periph_clock_enable(RCU_CAN0);
        break;
    // case CAN1:
    //     rcu_periph_clock_enable(RCU_CAN1); NO CAN1 on GD32F303VCT6
    //     break;
    default:
        return CAN_ERROR; // 不支持的 CAN 实例
    }

    rcu_periph_clock_enable(tx_gpio_rcu);
    if (tx_gpio_rcu != rx_gpio_rcu)
    {
        rcu_periph_clock_enable(rx_gpio_rcu);
    }
    rcu_periph_clock_enable(RCU_AF);

    // GPIO 初始化
    gpio_init(init->TX_GPIO_Port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, init->TX_Pin);
    gpio_init(init->RX_GPIO_Port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, init->RX_Pin);

    // 判断 remap
    if (init->TX_GPIO_Port == GPIOB && init->TX_Pin == GPIO_PIN_9 &&
        init->RX_GPIO_Port == GPIOB && init->RX_Pin == GPIO_PIN_8)
    {
        gpio_pin_remap_config(GPIO_CAN_PARTIAL_REMAP, ENABLE);
    }
    else if (init->TX_GPIO_Port == GPIOD && init->TX_Pin == GPIO_PIN_1 &&
             init->RX_GPIO_Port == GPIOD && init->RX_Pin == GPIO_PIN_0)
    {
        gpio_pin_remap_config(GPIO_CAN_FULL_REMAP, ENABLE);
    }
    else if (init->TX_GPIO_Port == GPIOA && init->TX_Pin == GPIO_PIN_12 &&
             init->RX_GPIO_Port == GPIOA && init->RX_Pin == GPIO_PIN_11)
    {
        // 默认引脚，无需 remap
    }
    else
    {
        return CAN_ERROR; // 不支持的引脚组合
    }

    // CAN 参数配置
    can_parameter_struct cp;
    can_struct_para_init(CAN_INIT_STRUCT, &cp);
    cp.working_mode = CAN_NORMAL_MODE;
    cp.resync_jump_width = CAN_BT_SJW_1TQ;
    cp.time_segment_1 = CAN_BT_BS1_7TQ;
    cp.time_segment_2 = CAN_BT_BS2_2TQ;
    cp.prescaler = rcu_clock_freq_get(CK_APB1) * 0.001 / (init->BaudrateKbps * (1 + 7 + 2)); // 60MHz APB1
    cp.auto_bus_off_recovery = ENABLE;
    cp.auto_retrans = ENABLE;
    cp.auto_wake_up = DISABLE;
    cp.rec_fifo_overwrite = DISABLE;
    cp.trans_fifo_order = DISABLE;
    cp.time_triggered = DISABLE;

    can_deinit(hcan->Instance);
    if (1 != can_init(hcan->Instance, &cp))
    {
        return CAN_ERROR; // 初始化失败
    }

    // CAN Filter 配置
    can_filter_parameter_struct cf;
    can_struct_para_init(CAN_FILTER_STRUCT, &cf);
    cf.filter_number = 0;
    cf.filter_mode = CAN_FILTERMODE_MASK;
    cf.filter_bits = CAN_FILTERBITS_32BIT;
    cf.filter_list_high = 0x0000;
    cf.filter_list_low = 0x0000;
    cf.filter_mask_high = 0x0000;
    cf.filter_mask_low = 0x0000;
    cf.filter_fifo_number = CAN_FIFO0;
    cf.filter_enable = ENABLE;
    can_filter_init(&cf);

    return CAN_OK;
}

rcu_periph_enum gpio_port_to_rcu(uint32_t port)
{
    switch (port)
    {
    case GPIOA:
        return RCU_GPIOA;
    case GPIOB:
        return RCU_GPIOB;
    case GPIOC:
        return RCU_GPIOC;
    case GPIOD:
        return RCU_GPIOD;
    case GPIOE:
        return RCU_GPIOE;
    default:
        return 0;
    }
}
