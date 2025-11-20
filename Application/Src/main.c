#include "main.h"  // IWYU pragma: export
#include "Initialization.h"
#include "com.h"
#include "hardware_interface.h"

bool pin = false;

/*!
    \brief      main function
*/
int main(void)
{
    Initialization_Drivers();
    Initialization_MTPA();
    while (1)
    {
        COM_CANProtocol();
        COM_SCIProtocol();
        // COM_DAQProtocol(systick_ms); Use CCP DAQ may cause PiSnoop display
        // offline
        Peripheral_Update_Temperature();
        Peripheral_Update_Break();

        pin = Peripheral_Get_HardwareBrk();
        // DWT_Count = DWT->CYCCNT; // 读取DWT计数器
    }
}
