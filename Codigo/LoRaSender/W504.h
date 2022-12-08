#include "Arduino.h"
#include "Modbus.h"
#include "ModbusSerial.h"

class W504 : public ModbusSerial{
  private:
    byte _slaveId;
    byte _ADDRESS;
    byte _FUNCTION;
    byte _START_ADD_LO;
    byte _START_ADD_HI;
    byte _NUM_REGISTERS_LO;
    byte _NUM_REGISTERS_HI;
    byte _NUM_BYTE; //N×2
    byte _REGISTER_DATA_LO;
    byte _REGISTER_DATA_HI;
    byte _CRC_HI;
    byte _CRC_LO;
  public:
    W504();
    void setSlaveID(byte slaveId);
    void setSlaveIDRequest(int DE, int RE);
    void startMeasurementRequest();
    void getTempODRequest();    
};


//definir trama

//byte START //idle for 3.5 characters lenght, tal vez sea mejor ponerlo como un arreglo de bit equivalente a los 3.5 byte
//byte ADDRESS 0x01
//byte FUNCTION 0x10 //0x10 -> write; 0x03 -> reading
//byte START_ADD_HI 0x30
//byte START_ADD_LO 0x00
//byte NUM_REGISTERS_HI 0x00
//byte NUM_REGISTERS_LO 0x01
//byte NUM_BYTE 0x02
//byte REGISTER_DATA_HI 0x14
//byte REGISTER_DATA_LO 0x00
//byte CRC_HI 0x99
//byte CRC_LO 0x53
//byte END //igual a START  


//--------Request Frame for Read registers----------------
// FUNCTION
// 0x03  

// START ADDRESS(START_ADD) //0x0000…0xffff
//  START_ADD_HI  0x00…0xff
//  START_ADD_LO  0x00…0xff

// NUM_REGISTERS //1…125 en hexadecimal 0x0001….0x007D
// NUM_REGISTERS_HI 0x00….0x00
// NUM_REGISTERS_LO 0x01….0x7D


//--------Request Frame for Read registers----------------
//--------Response Frame for Read registers----------------
// FUNCTION
// 0x03

// -N = NUM_REGISTERS-
// NUM_BYTE //number of registers from START_ADD to the address 
//            indicated by NUM_REGISTERS, as there is low and high
//            thats why there is a 2. 
// NUM_REGISTERS_LO*0x02 //N×2



// REGISTER_DATA //N × 2 bytes
// byte REGISTER_DATA[N*2]
// for(int i=0; i<N*2; i++)
// REGISTER_DATA[i]

//--------Response Frame for Read registers----------------

//*********************************************************
//*********************************************************

//--------Request Frame for Write registers----------------
// FUNCTION
// 0x10

// START ADDRESS(START_ADD) //0x0000…0xffff
//  START_ADD_HI  0x00…0xff
//  START_ADD_LO  0x00…0xff

// NUM_REGISTERS //0x0001….0x0078
// NUM_REGISTERS_HI 0x00….0x00
// NUM_REGISTERS_LO 0x01….0x78

// -N = NUM_REGISTERS-
// NUM_BYTE //number of registers from START_ADD to the address 
//            indicated by NUM_REGISTERS, as there is low and high
//            thats why there is a 2. 
// NUM_REGISTERS_LO*0x02 //N×2

// REGISTER_DATA //N × 2 bytes
// byte REGISTER_DATA[N*2]
// for(int i=0; i<N*2; i++)
// REGISTER_DATA[i]

//--------Request Frame for Write registers----------------
//--------Response Frame for Write registers----------------
// FUNCTION
// 0x10

// START ADDRESS(START_ADD) //0x0000…0xffff
//  START_ADD_HI  0x00…0xff
//  START_ADD_LO  0x00…0xff

// NUM_REGISTERS //1…123 en hexadecimal 0x0001….0x007B
// NUM_REGISTERS_HI 0x00….0x00
// NUM_REGISTERS_LO 0x01….0x7B

//--------Response Frame for Write registers----------------


// Procedure to Get DO Value
// Power ON
// Delay >= 4s
// getTempDO 
// -Repetir 10 veces-
// calculateDOprom

// Master device sends a “request frame” with a targeted slave address. When slave device
// responses, it has to put its own address in the “response frame”, so that master device
// knows where the response comes from.



// to verify CRC use https://crccalc.com/


// definition of idle time
//if (baud > 19200)
//{
//    T1_5 = 750; //microseconds
//    T3_5 = 1750; //microseconds
//}
//else 
//{
//    T1_5 = 15000000/baud; //microseconds
//    T3_5 = 35000000/baud; //microseconds
//}

// assuming T1_5 = 750 and T3_5 = 1750 and that the transmition of frames
// spends T1 = 500 taking we would have a total transmition time per frame of:
// T3_5 + T1 + T1_5 + T1 + T1_5 + T1 + T1_5 + T1 + T3_5
// 1750 + 500 + 750 + 500 + 750 + 500 + 750 + 500 + 1750 = 7750 microseconds
// 
