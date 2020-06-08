#include "liderhand.h"

static uint8_t b64_chr[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static uint8_t b64_int(uint8_t ch) {
    // ASCII to base64_int
    // 65-90  Upper Case  >>  0-25
    // 97-122 Lower Case  >>  26-51
    // 48-57  Numbers     >>  52-61
    // 43     Plus (+)    >>  62
    // 47     Slash (/)   >>  63
    // 61     Equal (=)   >>  64~
    if (ch==43)
    return 62;
    if (ch==47)
    return 63;
    if (ch==61)
    return 64;
    if ((ch>47) && (ch<58))
    return ch + 4;
    if ((ch>64) && (ch<91))
    return ch - 'A';
    if ((ch>96) && (ch<123))
    return (ch - 'a') + 26;
    return 0;
}

static std::string b64_encode(const std::vector<uint8_t>& in) {

    std::string out;
    out.resize(((4 * in.size() / 3) + 3) & ~3);
    unsigned int i=0, j=0, k=0, s[3];

    for (i=0;i<in.size();i++) {
        s[j++]=in[i];
        if (j==3) {
            out[k+0] = b64_chr[ s[0]>>2 ];
            out[k+1] = b64_chr[ ((s[0]&0x03)<<4)+((s[1]&0xF0)>>4) ];
            out[k+2] = b64_chr[ ((s[1]&0x0F)<<2)+((s[2]&0xC0)>>6) ];
            out[k+3] = b64_chr[ s[2]&0x3F ];
            j=0; k+=4;
        }
    }

    if (j) {
        if (j==1)
            s[1] = 0;
        out[k+0] = b64_chr[ s[0]>>2 ];
        out[k+1] = b64_chr[ ((s[0]&0x03)<<4)+((s[1]&0xF0)>>4) ];
        if (j==2)
            out[k+2] = b64_chr[ ((s[1]&0x0F)<<2) ];
        else
            out[k+2] = '=';
        out[k+3] = '=';
        k+=4;
    }

    //out[k] = '\0';

    return out;
}

static std::vector<uint8_t> b64_decode(const std::string& in) {

    std::vector<uint8_t> out;
    out.resize(in.length());

    unsigned int i=0, j=0, k=0, s[4];

    for (i=0;i<in.length();i++) {
        s[j++]=b64_int(in[i]);
        if (j==4) {
            out[k+0] = (s[0]<<2)+((s[1]&0x30)>>4);
            if (s[2]!=64) {
                out[k+1] = ((s[1]&0x0F)<<4)+((s[2]&0x3C)>>2);
                if ((s[3]!=64)) {
                    out[k+2] = ((s[2]&0x03)<<6)+(s[3]); k+=3;
                } else {
                    k+=2;
                }
            } else {
                k+=1;
            }
            j=0;
        }
    }

    out.resize(k);

    return out;
}

static uint8_t CRC8_CCITT_Calc(uint8_t inCrc, uint8_t inData)
{
   uint8_t i;
   uint8_t data;
   data = inCrc ^ inData;
   for ( i = 0; i < 8; i++ )
   {
        if (( data & 0x80 ) != 0 )
        {
            data <<= 1;
            data ^= 0x07;
        }
        else
        {
            data <<= 1;
        }
   }
   return data;
}

LiderHand::ErrorStatus LiderHand::ParseDataFromLiderHand(std::string data)
{
    if(data.length() > 0)//if any data
    {
        if(data[data.length()-1] == '\n')
        {
            data.resize(data.length()-1);
        }
    }else
    {
        return ERROR;
    }

    if(data.length() < 1)//if any data to be decoded
    {
        return ERROR;
    }

    std::vector<uint8_t> decoded = b64_decode(data);

    uint8_t CRC_Val = 0x00;
    for(int i=0; i<decoded.size() - 1; i++)
    {
        CRC_Val = CRC8_CCITT_Calc(CRC_Val, decoded[i]);
    }

    if(CRC_Val != decoded[decoded.size() -1]) //if CRC Value not valid
    {
        return ERROR;
    }

    /*if((decoded[3] * 9 + 4) != (decoded.size() - 1)) //if packet size not valid
    {
        return ERROR;
    }*/

    READ_SystemOperationMode = (SystemOperationMode_Type)decoded[0];
    READ_CalibrationProcedure = (CalibrationProcedure_Type)decoded[1];
    READ_CurrentError = (CurrentError_Type)decoded[2];

    MotorDrivers.resize(decoded[3]);

    int readPtr = 4;

    for(int i=0; i<decoded[3]; i++)
    {
        MotorDrivers[i].READ_FreeDrive = (FreeDrive_Type)(decoded[readPtr] & FreeDrive_EN);
        MotorDrivers[i].READ_Direction = (Direction_Type)(decoded[readPtr] & Dir_Positive);
        MotorDrivers[i].READ_MotorDriverOperation = (MotorDriverOperation_Type)(decoded[readPtr] & Operation_Fault);
        readPtr ++;

        ((uint8_t*)&(MotorDrivers[i].READ_PWM))[0] = decoded[readPtr]; readPtr++;
        ((uint8_t*)&(MotorDrivers[i].READ_PWM))[1] = decoded[readPtr]; readPtr++;
        ((uint8_t*)&(MotorDrivers[i].READ_PositionSet))[0] = decoded[readPtr]; readPtr++;
        ((uint8_t*)&(MotorDrivers[i].READ_PositionSet))[1] = decoded[readPtr]; readPtr++;
        ((uint8_t*)&(MotorDrivers[i].READ_Current))[0] = decoded[readPtr]; readPtr++;
        ((uint8_t*)&(MotorDrivers[i].READ_Current))[1] = decoded[readPtr]; readPtr++;
        MotorDrivers[i].READ_PositionCurrent_Count = decoded[readPtr]; readPtr++;
        for(int p=0; p<MotorDrivers[i].READ_PositionCurrent_Count; p++)
        {
            ((uint8_t*)&(MotorDrivers[i].READ_PositionCurrent[p]))[0] = decoded[readPtr]; readPtr++;
            ((uint8_t*)&(MotorDrivers[i].READ_PositionCurrent[p]))[1] = decoded[readPtr]; readPtr++;
        }
    }

    return SUCCESS;
}

std::string LiderHand::EncodePayload(std::vector<uint8_t> data)
{
    std::string out;

    uint8_t CRC_Val = 0x00;
    for(int i=0; i<data.size(); i++)
    {
        CRC_Val = CRC8_CCITT_Calc(CRC_Val, data[i]);
    }

    data.push_back(CRC_Val);

    out = b64_encode(data);
    out.append("\n");

    return out;
}

std::string LiderHand::PrepareDataEnableStatusUpdate()
{
    std::vector<uint8_t> data;
    data.push_back(FT232_CMD_EnableStatusUpdate);

    return EncodePayload(data);
}

std::string LiderHand::PrepareDataDisableStatusUpdate()
{
    std::vector<uint8_t> data;
    data.push_back(FT232_CMD_DisableStatusUpdate);

    return EncodePayload(data);
}

std::string LiderHand::PrepareDataPerformCalibration()
{
    std::vector<uint8_t> data;
    data.push_back(FT232_CMD_CalibrationProcedureEnable);

    return EncodePayload(data);
}

std::string LiderHand::PrepareDataResetErrors()
{
    std::vector<uint8_t> data;
    data.push_back(FT232_CMD_ResetErrors);

    return EncodePayload(data);
}

std::string LiderHand::PrepareDataIdleMode()
{
    std::vector<uint8_t> data;
    data.push_back(FT232_CMD_IdleMode);
    data.push_back(MotorDrivers.size());

    for(int i=0; i<MotorDrivers.size(); i++)
    {
        data.push_back((uint8_t)MotorDrivers[i].WRITE_FreeDrive);
    }

    return EncodePayload(data);
}

std::string LiderHand::PrepareDataInternalRegMode()
{
    std::vector<uint8_t> data;
    data.push_back(FT232_CMD_IntRegulatorMode);    
    data.push_back(MotorDrivers.size());

    for(int i=0; i<MotorDrivers.size(); i++)
    {
        data.push_back(((uint8_t*)&(MotorDrivers[i].WRITE_PositionSet))[0]);
        data.push_back(((uint8_t*)&(MotorDrivers[i].WRITE_PositionSet))[1]);
    }

    return EncodePayload(data);
}

std::string LiderHand::PrepareDataExternalRegMode()
{
    std::vector<uint8_t> data;
    data.push_back(FT232_CMD_ExtRegulatorMode);    
    data.push_back(MotorDrivers.size());

    for(int i=0; i<MotorDrivers.size(); i++)
    {
        data.push_back((uint8_t)MotorDrivers[i].WRITE_FreeDrive | (uint8_t)MotorDrivers[i].WRITE_Direction);
        data.push_back(((uint8_t*)&(MotorDrivers[i].WRITE_PWM))[0]);
        data.push_back(((uint8_t*)&(MotorDrivers[i].WRITE_PWM))[1]);
    }

    return EncodePayload(data);
}
