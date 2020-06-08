#include <QCoreApplication>
#include <QSerialPort>

#include <iostream>

#include "liderhand.h"

typedef enum
{
    IDLE,
    INTERNAL,
    EXTERNAL
}Mode_Type;

Mode_Type Mode;
QSerialPort serial;
LiderHand LiderHandObj;

void usage()
{
    std::cout << "\tLiderHandCommTest <serialport name> <mode>" << std::endl;
    std::cout << "\t\t<serialport name> - name of the serial port eg. COM3 or /dev/tty0" << std::endl;
    std::cout << "\t\t<mode> - mode of operation, can be value of: IDLE | INTERNAL | EXTERNAL" << std::endl;
    std::cout << std::endl;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    serial.setBaudRate(460800, QSerialPort::AllDirections);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);

    if(argc != 3)
    {
        std::cout << "Wrong argument count" << std::endl << std::endl;
        usage();
        return 0;
    }else
    {
        if(strcmp(argv[2], "IDLE") == 0)
        {
            Mode = IDLE;
        }else if(strcmp(argv[2], "INTERNAL") == 0)
        {
            Mode = INTERNAL;
        }else if(strcmp(argv[2], "EXTERNAL") == 0)
        {
            Mode = EXTERNAL;
        }else
        {
            std::cout << "Wrong <mode> argument value" << std::endl << std::endl;
            usage();
            return 0;
        }

        serial.setPortName(argv[1]);
    }

    if(serial.open(QIODevice::ReadWrite))//open serial port
    {
        std::cout << "Serial opened" << std::endl;

        //------EXAMPLE-------------------------------------------------------------------//
        //------FUNCTIONAL COMMANDS EXAMPLES----------------------------------------------//

        //------ENABLES LiderHand TO SEND STATUS UPDATED (100 Hz)-------------------------//
        std::string EnableStatusPayload = LiderHandObj.PrepareDataEnableStatusUpdate();
        serial.write(EnableStatusPayload.c_str(), EnableStatusPayload.length());
        serial.waitForBytesWritten(-1);

        //------DISABLES LiderHand TO SEND STATUS UPDATED---------------------------------//
        //std::string DisableStatusPayload = LiderHandObj.PrepareDataDisableStatusUpdate();
        //serial.write(DisableStatusPayload.c_str(), DisableStatusPayload.length());
        //serial.waitForBytesWritten(-1);

        //------LiderHand PERFORMS CALIBRATION OF ALL DRIVES------------------------------//
        //std::string CalibrationPayload = LiderHandObj.PrepareDataPerformCalibration();
        //serial.write(CalibrationPayload.c_str(), CalibrationPayload.length());
        //serial.waitForBytesWritten(-1);

        //------RESETS ALL LiderHand INTERNAL ERRORS, INCLUDING CurrentError STATUS AND---//
        //------Faul OPERATION OF ALL DRIVES - UNLOCKS FAULTY DRIVES----------------------//
        //std::string ResetErrPayload = LiderHandObj.PrepareDataResetErrors();
        //serial.write(ResetErrPayload.c_str(), ResetErrPayload.length());
        //serial.waitForBytesWritten(-1);


        while(1)
        {
            //------EXAMPLE-------------------------------------------------------------------//
            //------WAIT FOR NEW INCOMING DATA FROM LiderHand AND HANDLE THE DATA-------------//
            //------ALL DATA IS READ NO MATTER OF THE OPERATION MODE, YOU GET A FULL VECTOR---//
            //------FOR LiderHand TO START SENDING DATA, SEND THE EnableStatus COMMAND--------//

            serial.waitForReadyRead(-1);//wait for new data - blocking
            QByteArray data = serial.readLine();//read line (data is '\n' terminated)

            if(LiderHandObj.ParseDataFromLiderHand(data.toStdString()) == LiderHand::SUCCESS) //parse the input data and fill all the class content with current LiderHand data
            {
                uint8_t count = LiderHandObj.GetMotorDriverCount(); //acces driver count
                std::cout << "Read SUCCESS Drv count = " << (int)count << std::endl;

                LiderHand::SystemOperationMode_Type mode = LiderHandObj.GetSystemOperationMode(); //acces mode etc.
                LiderHand::CalibrationProcedure_Type calib = LiderHandObj.GetCalibrationProcedure();
                LiderHand::CurrentError_Type error = LiderHandObj.GetCurrentError();

                if(error != LiderHand::ERROR_OK)//system error handling (also indicated by LED blinking)
                {
                    if(error & LiderHand::ERROR_RS485_TIMEOUT)
                    {
                        std::cout << "LiderHand RS485 TIMEOUT ERROR" << std::endl;
                    }
                    if(error & LiderHand::ERROR_RS485_CRC)
                    {
                        std::cout << "LiderHand RS485 CRC ERROR" << std::endl;
                    }
                    if(error & LiderHand::ERROR_MOTOR_FAULT)
                    {
                        std::cout << "LiderHand MOTOR ERROR" << std::endl;
                    }
                    if(error & LiderHand::ERROR_FT232_CRC)
                    {
                        std::cout << "LiderHand PC CRC ERROR" << std::endl;
                    }
                }

                for(int i=0; i<count; i++)
                {
                    LiderHand::Direction_Type dir = LiderHandObj.GetDirection(i); //acces driver specyfic data
                    LiderHand::FreeDrive_Type fd = LiderHandObj.GetFreeDrive(i);
                    LiderHand::MotorDriverOperation_Type op = LiderHandObj.GetMotorDriverOperation(i);
                    uint16_t pwm = LiderHandObj.GetPWM(i);
                    uint16_t cur = LiderHandObj.GetCurrent(i);
                    uint8_t PositionCurrent_Count = LiderHandObj.GetPositonCurrent_Count(i);
                    uint16_t PositionCurrent[PositionCurrent_Count];
                    for(int p=0; p<PositionCurrent_Count; p++)
                    {
                        PositionCurrent[p] = LiderHandObj.GetPositonCurrent(i, p);
                    }
                    uint16_t posSet = LiderHandObj.GetPositonSet(i);

                    if(op == LiderHand::Operation_Fault) //handle particula driver error
                    {
                        std::cout << "Motor " << i << " faulty" << std::endl;
                    }
                }
            }else
            {
                std::cout << "Read ERROR" << std::endl;
            }

            //------EXAMPLE-------------------------------------------------------------------//
            //------IDLE - LiderHand DOES NOT PERFORM ANY ACTION, ALL DRIVES BREAK------------//
            //------YOU CAN CHOOSE IF EACH DRIVE IS IN FREEDRIVE MODE-------------------------//
            if(Mode == IDLE)
            {
                uint8_t DrvCount = LiderHandObj.GetMotorDriverCount();
                for(int i=0; i<DrvCount; i++)
                {
                    LiderHandObj.SetFreeDrive(i, LiderHand::FreeDrive_DIS); //is FreeDrive mode enabled
                }

                std::string IdlePayload = LiderHandObj.PrepareDataIdleMode();
                serial.write(IdlePayload.c_str(), IdlePayload.length());
                serial.waitForBytesWritten(-1);
            }

            //------EXAMPLE-------------------------------------------------------------------//
            //------INTERNAL - INTERNAL LiderHand REGULATOR IS USED---------------------------//
            //------ONLY A POSITION TO BE OBTAINED FOR EACH DRIVE IS SEND---------------------//
            //------LiderHand USES INTERNAL SIMPLE POSITION REGULATOR-------------------------//
            if(Mode == INTERNAL)
            {
                uint8_t DrvCount = LiderHandObj.GetMotorDriverCount();
                for(int i=0; i<DrvCount; i++)
                {
                    LiderHandObj.SetPosition(i, 30000); //set position of each drive here, range 1-65535
                }

                std::string InternalPayload = LiderHandObj.PrepareDataInternalRegMode();
                serial.write(InternalPayload.c_str(), InternalPayload.length());
                serial.waitForBytesWritten(-1);
            }

            //------EXAMPLE-------------------------------------------------------------------//
            //------EXTERNAL - USER SETS PWM OF EACH DRIVE, REGULATOR HAS TO BE IMPLEMENTED---//
            //------YOU HAVE TO USE SENSOR DATA OF EACH DRIVE AND CALCULATE THE PWM OF EACH---//
            //------DRIVE, USER HAS FULL CONTROL OVER ALL MOTOR-------------------------------//
            if(Mode == EXTERNAL)
            {
                uint8_t DrvCount = LiderHandObj.GetMotorDriverCount();
                for(int i=0; i<DrvCount; i++)
                {
                    uint16_t Current = LiderHandObj.GetCurrent(i);
                    uint8_t PositionCurrent_Count = LiderHandObj.GetPositonCurrent_Count(i);
                    uint16_t PositionCurrent[PositionCurrent_Count];
                    for(int p=0; p<PositionCurrent_Count; p++)
                    {
                        PositionCurrent[p] = LiderHandObj.GetPositonCurrent(i, p);
                    }
                    //----IMPLEMENT YOUR REGULATOR HERE---//

                    LiderHandObj.SetPWM(i, 0); //PWM value, range 1-65535
                    LiderHandObj.SetFreeDrive(i, LiderHand::FreeDrive_DIS); //is FreeDrive mode enabled
                    LiderHandObj.SetDirection(i, LiderHand::Dir_Positive);
                }

                std::string ExternalPayload = LiderHandObj.PrepareDataExternalRegMode();
                serial.write(ExternalPayload.c_str(), ExternalPayload.length());
                serial.waitForBytesWritten(-1);
            }
        }

    }else
    {
        std::cout << "Serial open fail" << std::endl;
    }

    return a.exec();
}
