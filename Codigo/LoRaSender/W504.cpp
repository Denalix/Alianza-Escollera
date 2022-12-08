#include "W504.h"


W504::W504() {
  
}

void W504::setSlaveID(byte slaveId)
{
  this->_slaveId = slaveId;
}

void W504::setSlaveIDRequest(int DE, int RE)
{
  this->_ADDRESS = this->_slaveId;
  this->_FUNCTION = 0x10; //0x10 -> write; 0x03 -> reading
  this->_START_ADD_LO = 0x00;
  this->_START_ADD_HI = 0x30;
  this->_NUM_REGISTERS_LO = 0x01;
  this->_NUM_REGISTERS_HI = 0x00;
  //this->_NUM_BYTE = this->_NUM_REGISTERS_LO*0x02; //N×2
  //this->_REGISTER_DATA_LO = 0x00;
  //this->_REGISTER_DATA_HI = 0x14;//set slave id=14
  this->_CRC_HI = 0x99;
  this->_CRC_LO = 0x53;
  
  digitalWrite(RE,HIGH);
  digitalWrite(DE,HIGH);
  
   //ID
  Serial.write(this->_ADDRESS);
  
  //Function code
  Serial.write(this->_FUNCTION);
  
  //Starting addres #1 MSB
  Serial.write(this->_START_ADD_HI);
  //Starting addres #2 LSB
  Serial.write(this->_START_ADD_LO);
  
  //Quantity to read #1 MSB
  Serial.write(this->_NUM_REGISTERS_HI);
  //Quantity to read #2 LSB
  Serial.write(this->_NUM_REGISTERS_LO);

  //CRC #1 MSB
  Serial.write(this->_CRC_HI);
  //CRC #2 LSB
  Serial.write(this->_CRC_LO);
  
  Serial.flush();
  delayMicroseconds(120);
  
  digitalWrite(RE,LOW);
  digitalWrite(DE,LOW);

  delay(1000);

}

void W504::startMeasurementRequest()
{
  this->_ADDRESS = this->_slaveId;
  this->_FUNCTION = 0x10; //0x10 -> write; 0x03 -> reading
  this->_START_ADD_LO = 0x00;
  this->_START_ADD_HI = 0x25;
  this->_NUM_REGISTERS_LO = 0x01;
  this->_NUM_REGISTERS_HI = 0x00;
  this->_NUM_BYTE = this->_NUM_REGISTERS_LO*0x02; //N×2
  this->_REGISTER_DATA_LO = 0x00;
  this->_REGISTER_DATA_HI = 0x14;//set slave id=14
  this->_CRC_HI = 0x06;
  this->_CRC_LO = 0x8F;
}

void W504::getTempODRequest()
{
  this->_ADDRESS = this->_slaveId;
  this->_FUNCTION = 0x03; //0x10 -> write; 0x03 -> reading
  this->_START_ADD_LO = 0x00;
  this->_START_ADD_HI = 0x26;
  this->_NUM_REGISTERS_LO = 0x06;
  this->_NUM_REGISTERS_HI = 0x00;
  this->_NUM_BYTE = this->_NUM_REGISTERS_LO*0x02; //N×2
  this->_REGISTER_DATA_LO = 0x00;
  this->_REGISTER_DATA_HI = 0x14;//set slave id=14
  this->_CRC_HI = 0xce;
  this->_CRC_LO = 0x80;
}
