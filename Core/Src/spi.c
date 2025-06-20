#include "spi.h"
#include "main.h"

uint8_t EXC_Freq = 0x28; // 10kHz;
uint8_t Control_Data = 0x7D; // 0x7D for 14bit Resolution, 16bit EncoderResolution
ErrStatus AD2S1210_Ready = SUCCESS;

void SPI_Init(void)
{
    spi_parameter_struct spi_init_struct;

    /* SPI2 use CK_APB1(60M) */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_SPI2);

    /* 配置 SPI2_SCK(PB3), SPI2_MOSI(PB5) 为推挽输出 */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3 | GPIO_PIN_5);
    /* 配置 SPI2_MISO(PB4) 为浮空输入 */
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_4);

    /* 软件 NSS，不使用硬件 NSS 引脚 */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE; // CKPH=1 CKPL=0
    spi_init_struct.nss                  = SPI_NSS_SOFT; // 软件 NSS
    spi_init_struct.prescale             = SPI_PSC_4;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI2, &spi_init_struct);

    /* 启用 SPI2 */
    spi_enable(SPI2);
}
extern uint16_t receive;

void AD2S1210_Init(void)
{

    gpio_init(A0PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, A0Pin);
    gpio_init(A1PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, A1Pin);
    gpio_init(RESETPORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, RESETPin);
    gpio_init(WRPORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, WRPin);
    gpio_init(SAMPLEPORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SAMPLEPin);
    gpio_bit_set(A0PORT, A0Pin);
    gpio_bit_set(A1PORT, A1Pin);
    gpio_bit_set(WRPORT, WRPin);
    gpio_bit_set(SAMPLEPORT, SAMPLEPin);
    gpio_bit_set(RESETPORT, RESETPin);

    //< Power up>//
    delay_ms(50);

    //< Reset >//
    gpio_bit_reset(RESETPORT, RESETPin);
    delay_ms(50);
    gpio_bit_set(RESETPORT, RESETPin);

    //< Enter configuration mode >//
    gpio_bit_set(A0PORT, A0Pin);
    gpio_bit_set(A1PORT, A1Pin);

    delay_us(1);

    //- Write Excitation -//
    gpio_bit_reset(WRPORT, WRPin); // WR must goes low before sending data
    spi_send_receive_byte(SPI2, EXCITE_REG); // Send EXCITE_REG
    gpio_bit_set(WRPORT, WRPin); // WR must goes high after sending data

    delay_us(1);

    gpio_bit_reset(WRPORT, WRPin); // Reset WR
    spi_send_receive_byte(SPI2, Excitation_Frequency); // Send Data Excitation frequency
    gpio_bit_set(WRPORT, WRPin); // Set WR

    delay_us(1);

    //- Read back while writing Control Register -//
    gpio_bit_reset(WRPORT, WRPin); // Reset WR
    if(Excitation_Frequency == spi_send_receive_byte(SPI2, CONTROL_REG))
    {
        AD2S1210_Ready &= SUCCESS;
    }else{
        AD2S1210_Ready &= ERROR;
    }
    gpio_bit_set(WRPORT, WRPin); // Set WR

    delay_us(1);

    gpio_bit_reset(WRPORT, WRPin); // Reset WR
    spi_send_receive_byte(SPI2, Control_Register_Data); // Send Data RES and EnRES
    gpio_bit_set(WRPORT, WRPin); // Set WR

    delay_us(1);

    //! Use the ERROR Register to read back Data and exit !//
    gpio_bit_reset(WRPORT, WRPin); // Reset WR
    if(Control_Register_Data == spi_send_receive_byte(SPI2, ERROR_REG))
    {
        AD2S1210_Ready &= SUCCESS;
    }else{
        AD2S1210_Ready &= ERROR;
    }
    gpio_bit_set(WRPORT, WRPin); // Set WR

    delay_us(1);

    //< Exit configuration mode >//
    gpio_bit_reset(A0PORT, A0Pin);
    gpio_bit_reset(A1PORT, A1Pin);

}

// uint16_t AD2S1210_Read(void)
// {

// }

uint8_t spi_send_receive_byte(uint32_t spi_periph, uint8_t byte)
{
    /* 等待发送缓冲区空 */
    while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_TBE));

    /* 发送字节 */
    spi_i2s_data_transmit(spi_periph, byte);

    /* 等待接收缓冲区非空 */
    while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_RBNE));

    /* 返回接收到的字节 */
    return spi_i2s_data_receive(spi_periph);
}
