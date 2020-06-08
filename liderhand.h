#ifndef LIDERHAND_H
#define LIDERHAND_H

#include <stdint.h>
#include <vector>
#include <string>

class LiderHand
{
public:
    LiderHand() {}

public:
    typedef enum{
        MODE_IDLE = 0x00,
        MODE_INT_REGULATOR = 0x01,
        MODE_EXT_REGULATOR = 0x02,
    }SystemOperationMode_Type;

    typedef enum{
        CALIBRATION_Disabled,
        CALIBRATION_Perform
    }CalibrationProcedure_Type;

    typedef enum
    {
        ERROR_OK = 0x00,
        ERROR_RS485_TIMEOUT = 0x01,
        ERROR_MOTOR_FAULT = 0x02,
        ERROR_RS485_CRC = 0x04,
        ERROR_FT232_CRC = 0x08,
    }CurrentError_Type;

    typedef enum
    {
        Dir_Positive =  0x02,
        Dir_Negative = 0x00,
        Dir_Invalid = -0x01
    }Direction_Type;

    typedef enum
    {
        FreeDrive_EN  = 0x01,
        FreeDrive_DIS = 0x00,
        FreeDrive_Invalid = -0x01
    }FreeDrive_Type;

    typedef enum
    {
        Operation_OK = 0x00,
        Operation_Fault = 0x04,
        Operation_Invalid = -0x01
    }MotorDriverOperation_Type;

    typedef enum
    {
        FaultFlag_Keep = 0x00,
        FaultFlag_Reset = 0x04
    }ResetFaultFlag_Type;

    typedef enum
    {
      ERROR = 0,
      SUCCESS = !ERROR
    }ErrorStatus;

private:
    #define PositionCurrent_Count_Max			4

    typedef struct
    {
        uint16_t                    READ_PWM                    = 0;
        uint16_t                    WRITE_PWM                   = 0;

        Direction_Type              READ_Direction              = Dir_Positive;
        Direction_Type              WRITE_Direction             = Dir_Positive;

        FreeDrive_Type              READ_FreeDrive              = FreeDrive_DIS;
        FreeDrive_Type              WRITE_FreeDrive             = FreeDrive_DIS;

        uint16_t                    READ_Current                = 0;

        uint8_t                     READ_PositionCurrent_Count  = 1;

        uint16_t                    READ_PositionCurrent[PositionCurrent_Count_Max];

        uint16_t                    READ_PositionSet            = 32767;
        uint16_t                    WRITE_PositionSet           = 32767;

        MotorDriverOperation_Type   READ_MotorDriverOperation   = Operation_OK;
    }MotorDriver_Type;

    typedef enum
    {
        FT232_CMD_EnableStatusUpdate = 0x01,
        FT232_CMD_DisableStatusUpdate = 0x02,
        FT232_CMD_CalibrationProcedureEnable = 0x03,
        FT232_CMD_ResetErrors = 0x04,
        FT232_CMD_IdleMode = 0x05,
        FT232_CMD_IntRegulatorMode = 0x06,
        FT232_CMD_ExtRegulatorMode = 0x07
    }FT232_CMD_Type;

private:
    SystemOperationMode_Type        READ_SystemOperationMode    = MODE_IDLE;
    CalibrationProcedure_Type       READ_CalibrationProcedure   = CALIBRATION_Disabled;
    CurrentError_Type               READ_CurrentError           = ERROR_OK;

    std::vector<MotorDriver_Type>   MotorDrivers;

    std::string                 EncodePayload(std::vector<uint8_t> data);

public:
    void                        DummyInit(uint8_t drvCount)         {MotorDrivers.resize(drvCount);}

    ErrorStatus                 ParseDataFromLiderHand(std::string data);

    std::string                 PrepareDataEnableStatusUpdate();
    std::string                 PrepareDataDisableStatusUpdate();
    std::string                 PrepareDataPerformCalibration();
    std::string                 PrepareDataResetErrors();

    std::string                 PrepareDataIdleMode();
    std::string                 PrepareDataInternalRegMode();
    std::string                 PrepareDataExternalRegMode();

    uint8_t                     GetMotorDriverCount()                       {return MotorDrivers.size();}
    SystemOperationMode_Type    GetSystemOperationMode()                    {return READ_SystemOperationMode;}
    CalibrationProcedure_Type   GetCalibrationProcedure()                   {return READ_CalibrationProcedure;}
    CurrentError_Type           GetCurrentError()                           {return READ_CurrentError;}

    Direction_Type              GetDirection(uint8_t drv)                   {if(drv < MotorDrivers.size()) {return MotorDrivers[drv].READ_Direction;}else{return Dir_Invalid;}}
    FreeDrive_Type              GetFreeDrive(uint8_t drv)                   {if(drv < MotorDrivers.size()) {return MotorDrivers[drv].READ_FreeDrive;}else{return FreeDrive_Invalid;}}
    MotorDriverOperation_Type   GetMotorDriverOperation(uint8_t drv)        {if(drv < MotorDrivers.size()) {return MotorDrivers[drv].READ_MotorDriverOperation;}else{return Operation_Invalid;}}
    uint16_t                    GetPWM(uint8_t drv)                         {if(drv < MotorDrivers.size()) {return MotorDrivers[drv].READ_PWM;}else{return 0;}}
    uint16_t                    GetCurrent(uint8_t drv)                     {if(drv < MotorDrivers.size()) {return MotorDrivers[drv].READ_Current;}else{return 0;}}
    uint8_t                     GetPositonCurrent_Count(uint8_t drv)        {if(drv < MotorDrivers.size()) {return MotorDrivers[drv].READ_PositionCurrent_Count;}else{return 0;}}
    uint16_t                    GetPositonCurrent(uint8_t drv, uint8_t enc) {
                                                                                if(drv < MotorDrivers.size())
                                                                                {
                                                                                    if(enc < MotorDrivers[drv].READ_PositionCurrent_Count)
                                                                                    {
                                                                                        return MotorDrivers[drv].READ_PositionCurrent[enc];
                                                                                    }
                                                                                }
                                                                                return 0;
                                                                            }
    uint16_t                    GetPositonSet(uint8_t drv)                  {if(drv < MotorDrivers.size()) {return MotorDrivers[drv].READ_PositionSet;}else{return 0;}}

    bool                        SetDirection(uint8_t drv, Direction_Type val)   {if(drv < MotorDrivers.size()) {MotorDrivers[drv].WRITE_Direction = val; return true;}else{return false;}}
    bool                        SetFreeDrive(uint8_t drv, FreeDrive_Type val)   {if(drv < MotorDrivers.size()) {MotorDrivers[drv].WRITE_FreeDrive = val; return true;}else{return false;}}
    bool                        SetPWM(uint8_t drv, uint16_t val)               {if(drv < MotorDrivers.size()) {MotorDrivers[drv].WRITE_PWM = val;; return true;}else{return false;}}
    bool                        SetPosition(uint8_t drv, uint16_t val)          {if(drv < MotorDrivers.size()) {MotorDrivers[drv].WRITE_PositionSet = val;; return true;}else{return false;}}
};

#endif // LIDERHAND_H
