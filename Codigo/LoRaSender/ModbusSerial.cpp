/*
    ModbusSerial.cpp - Source for Modbus Serial Library
    Copyright (C) 2014 André Sarmento Barbosa
*/
#include "ModbusSerial.h"

ModbusSerial::ModbusSerial() {

}

bool ModbusSerial::setSlaveId(byte slaveId){
    _slaveId = slaveId;
    return true;
}

byte ModbusSerial::getSlaveId() {
    return _slaveId;
}

bool ModbusSerial::config(HardwareSerial* port, long baud, u_int format, int txPin) {
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

#ifdef USE_SOFTWARE_SERIAL
bool ModbusSerial::config(SoftwareSerial* port, long baud, int txPin) {
    this->_port = port;
    this->_txPin = txPin;
    (*port).begin(baud);

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
#endif

#ifdef __AVR_ATmega32U4__
bool ModbusSerial::config(Serial_* port, long baud, u_int format, int txPin) {
    this->_port = port;
    this->_txPin = txPin;
    (*port).begin(baud, format);
    while (!(*port));

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
#endif

bool ModbusSerial::receive(byte* frame) {// TERCER PUNTO (3.1)
    //first byte of frame = address
    byte address = frame[0]; // DEFINIR EL ADDRESS COMO EL FRAME[0] O EL ID DEL ESCLAVO (3.2)
    //Last two bytes = crc
    u_int crc = ((frame[_len - 2] << 8) | frame[_len - 1]);// DEFINIR LOS ÚLTIMOS DOS VALORES DEL FRAME COMO EL CRC DE TIPO UNSIGNED INT (3.3)

    //Slave Check
    if (address != 0xFF && address != this->getSlaveId()) {// CHECA SI LA DIRECCIÓN ES EL MISMO ESCLAVO (3.4)
		return false;
	}

    //CRC Check
    if (crc != this->calcCrc(_frame[0], _frame+1, _len-3)) {// CHECA SI EL CRC ES IGUAL AL CALCULO QUE SE HACE CON EL SLAVE ID, EL FRAME CORRIDO PARA QUE SEA SOLO EL PDU Y LA LONGITUD  (3.5)
		return false;
    }

    //PDU starts after first byte
    //framesize PDU = framesize - address(1) - crc(2)
    this->receivePDU(frame+1); // FUNCIÓN DE MODBUS.CPP  (3.6)
    //DADO QUE EN LA FUNCIÓN EN MODBUS.CPP EL FRAME[0] ES LA FUNCION EL MÁS UNO PUEDE SIGNIFICAR QUE SE ELIMINA EL FRAME[0] INICIAL QUE ERA EL ID DEL ESCLAVO
    //No reply to Broadcasts
    if (address == 0xFF) _reply = MB_REPLY_OFF;// (3.7.4)
    return true;
}

bool ModbusSerial::send(byte* frame) {// PRIMER PUNTO (1.1)
  //used to send reply after receiving
    byte i;

    if (this->_txPin >= 0) {
        digitalWrite(this->_txPin, HIGH); // ENCIENDE EL PIN DE TRANSMISIÓN DE/RE (1.2)
        delay(1);
    }

    for (i = 0 ; i < _len ; i++) {
        (*_port).write(frame[i]); // ESCRIBE UNO A UNO LOS BYTES DEL FRAME CON WRITE (1.3)
    }

    (*_port).flush();
    delayMicroseconds(_t35);

    // FORMA DE LA TRAMA HASTA AQUÍ: |FRAME|_t35

    if (this->_txPin >= 0) {
        digitalWrite(this->_txPin, LOW); // APAGA EL PIN DE TRANSMISIÓN DE/RE (1.4)
    }
}

bool ModbusSerial::sendPDU(byte* pduframe) {// SEGUNDO PUNTO (2.1)
  //used to write
    if (this->_txPin >= 0) {
        digitalWrite(this->_txPin, HIGH); // ENCIENDE EL PIN DE TRANSMISIÓN DE/RE (2.2)
        delay(1);
    }

    //Send slaveId
    (*_port).write(_slaveId); // ENVÍA EL ID DEL ESCLAVO (2.3)

    //Send PDU
    byte i;
    for (i = 0 ; i < _len ; i++) {
        (*_port).write(pduframe[i]); // ENVÍA UNO A UNO LOS BYTES DEL PDUFRAME (2.4)
    }

    //Send CRC
    word crc = calcCrc(_slaveId, _frame, _len); // CALCULA EL CRC (2.5)
    // as write is sending a parameter val it can only send one byte at a time
    (*_port).write(crc >> 8);// right shift, sends high // ENVÍA EL HIGH DEL CRC (2.5.1)
    (*_port).write(crc & 0xFF); //bitwise AND, send low // ENVÍA EL LOW DEL CRC (2.5.2)

    (*_port).flush();//Waits for the transmission of outgoing serial data to complete.
    delayMicroseconds(_t35);

    // FORMA DE LA TRAMA HASTA AQUÍ: |ID|PDU|CRC|_t35
    
    if (this->_txPin >= 0) {
        digitalWrite(this->_txPin, LOW);// APAGA EL PIN DE TRANSMISIÓN DE/RE (1.4)
    }
}

void ModbusSerial::task() {
    _len = 0;

    while ((*_port).available() > _len)	{ //Get the number of bytes (characters) available for reading from the serial port. 
        _len = (*_port).available();
        delayMicroseconds(_t15);
    }

    if (_len == 0) return;

    byte i;
    _frame = (byte*) malloc(_len);//dynamic memory allocation
    for (i=0 ; i < _len ; i++) _frame[i] = (*_port).read(); //read() Returns The first byte of incoming serial data available (or -1 if no data is available) - int.

    if (this->receive(_frame)) {// AHORA VAMOS A RECEIVE (3)
        if (_reply == MB_REPLY_NORMAL)
            this->sendPDU(_frame); // AHORA VAMOS AL SENDPDU (2)
        else
        if (_reply == MB_REPLY_ECHO)
            this->send(_frame);//EMPEZAR DESDE EL SEND (1)
    }

    free(_frame);
    _len = 0;
}

word ModbusSerial::calcCrc(byte address, byte* pduFrame, byte pduLen) {
	byte CRCHi = 0xFF, CRCLo = 0x0FF, Index;

    Index = CRCHi ^ address;
    CRCHi = CRCLo ^ _auchCRCHi[Index];
    CRCLo = _auchCRCLo[Index];

    while (pduLen--) {
        Index = CRCHi ^ *pduFrame++;
        CRCHi = CRCLo ^ _auchCRCHi[Index];
        CRCLo = _auchCRCLo[Index];
    }

    return (CRCHi << 8) | CRCLo;//shift left the high and bitwise XOR
}
