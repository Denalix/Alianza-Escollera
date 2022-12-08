#include "W520.h"

W520::W520() {

}

void W520::setSlaveID(byte slaveId)
{
  this->_slaveId = slaveId;
}


bool W520::config(HardwareSerial* port, long baud, u_int format, int txPin) {
    this->_port = port;
    this->_txPin = txPin;
    (*port).begin(baud, format);

    delay(2000);

    if (txPin >= 0) {
        pinMode(txPin, OUTPUT);
        digitalWrite(txPin, LOW);
    }

    if (baud > 19200) {
        _t15 = 750;
        _t35 = 1750;
    } else {
        _t15 = 15000000/baud; // 1T * 1.5 = T1.5
        _t35 = 35000000/baud; // 1T * 3.5 = T3.5
    }

    return true;
}


void W520::setSlaveIDRequest()
{
  this->_ADDRESS = this->_slaveId;
  this->_FUNCTION = 0x03; //0x10 -> write; 0x03 -> reading
  this->_START_ADD_LO = 0x00;
  this->_START_ADD_HI = 0x90;
  this->_NUM_REGISTERS_LO = 0x07;
  this->_NUM_REGISTERS_HI = 0x00;
  this->_NUM_BYTE = this->_NUM_REGISTERS_LO * 0x02; //N×2
  this->_REGISTER_DATA_LO = 0x00;
  this->_REGISTER_DATA_HI = 0x14;//set slave id=14
  this->_CRC_HI = 0x07;
  this->_CRC_LO = 0x94;
}

void W520::startMeasurementRequest()
{
  this->_ADDRESS = this->_slaveId;
  this->_FUNCTION = 0x03; //0x10 -> write; 0x03 -> reading
  this->_START_ADD_LO = 0x00;
  this->_START_ADD_HI = 0x90;
  this->_NUM_REGISTERS_LO = 0x07;
  this->_NUM_REGISTERS_HI = 0x00;
  this->_NUM_BYTE = this->_NUM_REGISTERS_LO * 0x02; //N×2
  this->_REGISTER_DATA_LO = 0x00;
  this->_REGISTER_DATA_HI = 0x14;//set slave id=14
  this->_CRC_HI = 0x07;
  this->_CRC_LO = 0x94;
}

int W520::getSerialNumberRequest(int DE, int RE, byte* ByteArray, unsigned int t_15, unsigned int t_35)
{
  this->_ADDRESS = this->_slaveId;
  this->_FUNCTION = 0x03; //0x10 -> write; 0x03 -> reading
  this->_START_ADD_LO = 0x00;
  this->_START_ADD_HI = 0x90;
  this->_NUM_REGISTERS_LO = 0x07;
  this->_NUM_REGISTERS_HI = 0x00;
//  this->_NUM_BYTE = this->_NUM_REGISTERS_LO * 0x02; //N×2
//  this->_REGISTER_DATA_LO = 0x00;
//  this->_REGISTER_DATA_HI = 0x14;//set slave id=14
//  this->_CRC_HI = 0x07;
//  this->_CRC_LO = 0x94;
  byte* _requestFramePDU;
  byte _len = 5;//longitud del requestFrame

  _requestFramePDU = (byte*) malloc(_len);//dynamic memory allocation
  _requestFramePDU[0] = this->_FUNCTION;
  _requestFramePDU[1] = this->_START_ADD_HI;
  _requestFramePDU[2] = this->_START_ADD_LO;
  _requestFramePDU[3] = this->_NUM_REGISTERS_HI;
  _requestFramePDU[4] = this->_NUM_REGISTERS_LO;

  word crc = calcCrc(_ADDRESS, _requestFramePDU, _len+1);//byte address, byte* pduFrame, byte pduLen
  
  //se comienza con el envío del request
  digitalWrite(DE, HIGH);

  delayMicroseconds(t_35);

  //ID
  (*_port).write(_ADDRESS);

  //Function code
  (*_port).write(_requestFramePDU[0]);

  //Starting addres #1 MSB
  (*_port).write(_requestFramePDU[1]);
  //Starting addres #2 LSB
  (*_port).write(_requestFramePDU[2]);

  //Quantity to read #1 MSB
  (*_port).write(_requestFramePDU[3]);
  //Quantity to read #2 LSB
  (*_port).write(_requestFramePDU[4]);

  //CRC #1 MSB
  (*_port).write(crc >> 8);// right shift, sends high
  //CRC #2 LSB
  (*_port).write(crc & 0xFF);//bitwise AND, send low

  delayMicroseconds(t_35);

  (*_port).flush();
  delayMicroseconds(120);

  //inicia la lectura de datos
  digitalWrite(RE, LOW);
  
  int a = 0;
  while ((*_port).available()) {
    ByteArray[a] = (*_port).read();
    a++;
  }

  free(_requestFramePDU);

  return a;
}


int W520::getTempCondRequest(int DE, int RE, byte* ByteArray, unsigned int t_15, unsigned int t_35)
{
  this->_ADDRESS = this->_slaveId;
  this->_FUNCTION = 0x03; //0x10 -> write; 0x03 -> reading
  this->_START_ADD_HI = 0x26;
  this->_START_ADD_LO = 0x00;
  this->_NUM_REGISTERS_HI = 0x00;
  this->_NUM_REGISTERS_LO = 0x05;
  //this->_CRC_HI = 0x07;
  //this->_CRC_LO = 0x94;
  byte* _requestFramePDU;
  byte _len = 5;//longitud del requestFrame

  _requestFramePDU = (byte*) malloc(_len);//dynamic memory allocation
  _requestFramePDU[0] = this->_FUNCTION;
  _requestFramePDU[1] = this->_START_ADD_HI;
  _requestFramePDU[2] = this->_START_ADD_LO;
  _requestFramePDU[3] = this->_NUM_REGISTERS_HI;
  _requestFramePDU[4] = this->_NUM_REGISTERS_LO;

  word crc = calcCrc(_ADDRESS, _requestFramePDU, _len+1);//byte address, byte* pduFrame, byte pduLen
  
  //se comienza con el envío del request
  digitalWrite(DE, HIGH);

  delayMicroseconds(t_35);

  //ID
  (*_port).write(_ADDRESS);

  //Function code
  (*_port).write(_requestFramePDU[0]);

  //Starting addres #1 MSB
  (*_port).write(_requestFramePDU[1]);
  //Starting addres #2 LSB
  (*_port).write(_requestFramePDU[2]);

  //Quantity to read #1 MSB
  (*_port).write(_requestFramePDU[3]);
  //Quantity to read #2 LSB
  (*_port).write(_requestFramePDU[4]);

  //CRC #1 MSB
  (*_port).write(crc >> 8);// right shift, sends high
  //CRC #2 LSB
  (*_port).write(crc & 0xFF);//bitwise AND, send low

  delayMicroseconds(t_35);

  (*_port).flush();
  delayMicroseconds(120);

  digitalWrite(RE, LOW);
  
  int a = 0;
  while ((*_port).available()) {
    ByteArray[a] = (*_port).read();
    a++;
  }

  free(_frame);

  return a;
}

word W520::calcCrc(byte address, byte* pduFrame, byte pduLen)
{
  byte CRCHi = 0xFF, CRCLo = 0x0FF, Index;

  Index = CRCHi ^ address; //bitwise XOR para address 0x01 es igual a 0x0E o 14
  CRCHi = CRCLo ^ _auchCRCHi[Index];//(0xFF XOR 0x81)=(0000 1111 XOR 1000 0001) = (1000 1110) = 0x8E
  CRCLo = _auchCRCLo[Index];//0xC4

  while (pduLen--) {
    Index = CRCHi ^ *pduFrame++;
    CRCHi = CRCLo ^ _auchCRCHi[Index];//(0x81)
    CRCLo = _auchCRCLo[Index];
  }

  return (CRCHi << 8) | CRCLo;//shift left the high and bitwise XOR
}
