#include "main_int.h"
#include "Buffer.h"
#include "Initialization.h"
#include "foc.h"
#include "hardware_interface.h"
#include "justfloat.h"
#include "leso.h"
#include "reciprocal.h"
#include "sensorless_interface.h"
#include "transformation.h"

static DeviceStateEnum_t MainInt_State        = RUNNING;
static volatile bool     MainInt_UseRealTheta = true;
//static volatile uint16_t MainInt_DataFlag     = 0x000U;

static inline void MainInt_Update_FocCurrent(void)
{
    Phase_t current_phase = Peripheral_Get_PhaseCurrent();
    Clark_t current_clark = {0};

    current_clark = ClarkeTransform(current_phase);
    current_clark = Sensorless_FilterCurrent(current_clark);
    Sensorless_Set_Current(current_clark);
    Foc_Set_Iclark_Fdbk(current_clark);
}

static inline void MainInt_Check_ProtectFlag(void)
{
    bool stop = Peripheral_Update_Break();
    Foc_Set_ResetFlag(stop);
    stop = Foc_Get_ResetFlag();
    Peripheral_Set_Stop(stop);
    Sensorless_Set_ResetFlag(stop);
}

static inline void MainInt_Update_BusVoltage(void)
{
    FloatWithInv_t bus_voltage = Peripheral_UpdateUdc();
    Foc_Set_BusVoltage(bus_voltage.val);
    Foc_Set_BusVoltageInv(bus_voltage.inv);
}

static inline void MainInt_Update_Angle_and_Speed(void)
{
    AngleResult_t res          = {0};
    AngleResult_t est          = {0};
    AngleResult_t real         = {0};
    float         speed_ref    = Foc_Get_SpeedRamp();
    float         speed_target = Foc_Get_SpeedTarget();

    real = Peripheral_Update_Position();
    est  = Sensorless_Update_Position();

    Sensorless_Calculate_Err(real);

    if (MainInt_UseRealTheta)
    {
        res = real;
    }
    else
    {
        res = est;
    }

    Foc_Set_Speed(res.speed);
    Foc_Set_Angle(res.theta);

    Sensorless_Set_SpeedRef(speed_ref);
    Sensorless_Set_SpeedTarget(speed_target);
    Sensorless_Set_SpeedFdbk(res.speed);
    //Sensorless_Set_Angle(res.theta);

    Buffer_Put(real.theta, 0);
    Buffer_Put(est.theta, 1);
    Buffer_Put(real.speed, 2);
    Buffer_Put(est.speed, 3);
    Buffer_Put(Sensorless_Get_Error().theta, 4);
}

static inline void MainInt_Initialization(void)
{
    Initialization_Modules();
    Peripheral_CalibrateADC();
    if (Foc_Get_BusVoltage() > 200.0F)
    {
        Peripheral_EnableHardwareProtect();
    }
    Peripheral_Reset_ProtectFlag();
    Foc_Set_Mode(IDLE);
}

static inline void MainInt_Startup(void)
{
    if (Sensorless_Get_Method() & FLYING)
    {
        Foc_Set_Mode(STARTUP);
    }
    else
    {
        Foc_Set_Mode(SPEED);
    }
}

static inline void MainInt_Update_Sensorless(void)
{
    Clark_t voltage = {0};
    Park_t  ref     = {0};
    Park_t  induc   = Foc_Get_Inductor();
    Leso_Set_Inductor(induc);

    voltage = Foc_Get_Uclark_Ref();

    Sensorless_Set_Voltage(voltage);

    Sensorless_Calculate();

    ref = Foc_Get_Udq_Ref();
    ref = Sensorless_Inject_Voltage(ref);
    Foc_Set_Udq_Ref(ref);
}

static inline void MainInt_Run_Foc(void)
{
    Park_t vol_dq_ref = {0};

    vol_dq_ref = Foc_Update_Main();

    Foc_Set_Udq_Ref(vol_dq_ref);
}

static inline void MainInt_Send_Data(void)
{
    Buffer_Send();
}

static inline void MainInt_Exit(void)
{
    Peripheral_Set_Stop(true);       // 停止所有操作
    Foc_Set_ResetFlag(true);         // 设置复位标志
    Foc_Set_Mode(IDLE);              // 切换到IDLE模式
    Peripheral_Reset_ProtectFlag();  // 重置保护标志
    Peripheral_DisableHardwareProtect();
}

static inline void MainInt_SVPWM(void)
{
    Phase_t tcm = Foc_Get_Tcm();  // 获取三相PWM时间
    Peripheral_Set_PWMChangePoint(tcm);
}

/*!
    \brief      主中断函数
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Main_Int_Handler(void)
{
    MainInt_Update_FocCurrent();
    MainInt_Update_Angle_and_Speed();
    MainInt_Update_BusVoltage();
    MainInt_Check_ProtectFlag();

    switch (MainInt_State)
    {
    case INIT:
    {
        MainInt_Initialization();
        MainInt_State = RUNNING;
        break;
    }

    case RUNNING:
    {
        // 运行状态：正常FOC控制
        MainInt_Run_Foc();
        MainInt_Update_Sensorless();
        break;
    }

    case SENSORLESS:
    {
        MainInt_Startup();
        MainInt_Run_Foc();
        MainInt_Update_Sensorless();
        break;
    }

    case EXIT:
    {
        // 退出状态：进行必要的清理
        MainInt_Exit();
        break;
    }
    }

    MainInt_SVPWM();
    MainInt_Send_Data();
}
