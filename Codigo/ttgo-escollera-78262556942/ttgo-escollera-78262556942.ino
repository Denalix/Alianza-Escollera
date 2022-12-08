

// References:
// [feather] adafruit-feather-m0-radio-with-lora-module.pdf


#include <lmic.h>
#include <hal/hal.h>
#include <LoRa.h>
#include "boards.h"
#include <ModbusMaster.h>               //Library for using ModbusMaster
#include <HardwareSerial.h>

#include <ArduinoJson.h>
StaticJsonDocument<128> doc;

#define batteryReadingPin 2
#define MAX485_DE_RE  13  // Connect RE an DE terminals with pin 2 of TTGO T-beam   
#define MAX485EnergyPin 14
#define sensorsEnergyPin 25 //

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// BOYA 3 BOYA 3  BOYA 3  BOYA 3  BOYA 3  BOYA 3  BOYA 3  BOYA 3  BOYA 3  BOYA 3  BOYA 3  BOYA 3  BOYA 3  BOYA 3  BOYA 3  BOYA 3  BOYA 3  BOYA 3  BOYA 3  BOYA 3  BOYA 3 BOYA 3  
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
int port_numBoya = 3;

const uint16_t _u16BaudRate = 9600;// is seems it only works with 9600
const uint8_t idConductivitySensor = 2;
const uint8_t idDOSensor = 1;
const uint8_t SFPowerOn = 3;// security factor for power on time
const int conducSensorpowerOnDelay = 10000 * SFPowerOn; //miliseconds
const int DOSensorpowerOnDelay = 4000; //miliseconds
const int conducSensorMeasureDelay = 3000;//miliseconds
const int DOSensorMeasureDelay = 1000;//miliseconds

//Conductivity sensor registers
const uint16_t regSerialNumber[] = {0x0900, 0x0007, 0x0013};//startAddres|numberOfRegisters|numberOfResponseBytes
const uint16_t regTempCond[] = {0x2600, 0x0005, 0x000F};

//DO sensor registers
const uint16_t regTempDO[] = {0x2600, 0x0006, 0x0011};
const uint16_t regSalinitySet[] = {0x1500, 0x0002, 0x0011};//%. The default value is 0
const uint16_t regAtmPressureSet[] = {0x2400, 0x0002, 0x0008};//kPa. The default value is 101.325kpa
const uint16_t regSlaveIdGet[] = {0x3000, 0x0001, 0x0007};
const uint16_t regSlaveIdSet[] = {0x3000, 0x0001, 0x0008};
const uint16_t regCalibrationCoefficientsSet[] = {0x1100, 0x0004, 0x0008};
const uint16_t regCalibrationCoefficientsGet[] = {0x1100, 0x0004, 0x000D};

//DO default setting
const float pressureDefault = 101.325;// Kpa
const float salinityDefault = 0.0; // ppt or ‰
const float KDefault = 1.0; // Calibration coefficients
const float BDefault = 0.0;

int absoluteCount = 0;
//int LORACounter = 0;
int counterFalseReading = 0;
const int totalNumMedidas = 15;
int averageCorrectionCond = 0;
int averageCorrectionDO = 0;
int t_0;
int t_f;
int t_f2;
int t_transmition = 122;//time spend between transmitions [seconds]: 122.037. From experimentation
int t_process = 60*5*1000;// constant time the process should take, miliseonds
int t_processInterval = t_process - t_transmition;



float tempArrayCond[totalNumMedidas];
float condArray[totalNumMedidas];
float tempArrayDO[totalNumMedidas];
float DOpArray[totalNumMedidas];//percentage array
float DOvArray[totalNumMedidas];//value array

float tempAverageCond;
float condAverage;
float salinityAverage;
float tempAverageDO;
float DOpAverage;
float DOvAverage;//value DO
float DOpvAverage;//value DO based on percentage
float tempAverage;//temp average of tempAverageCond and tempAverageDO
float voltInput;

ModbusMaster conductivitySensor;                    //object conductivitySensor for class ModbusMaster
ModbusMaster DOSensor;                    //object conductivitySensor for class ModbusMaster

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//

// 78262556942
// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
//                                          AF    F2    DE    CF    26    15    CF    A6    01    10    CB    68    CA    E4    F0    63
static const PROGMEM u1_t NWKSKEY[16] = { 0xAF, 0xF2, 0xDE, 0xCF, 0x26, 0x15, 0xCF, 0xA6, 0x01, 0x10, 0xCB, 0x68, 0xCA, 0xE4, 0xF0, 0x63};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
//                                          07    18    C8    74    99    85    66    7E    30    C7    DC    CD    D8    20    14    23
static const u1_t PROGMEM APPSKEY[16] = { 0x07, 0x18, 0xC8, 0x74, 0x99, 0x85, 0x66, 0x7E, 0x30, 0xC7, 0xDC, 0xCD, 0xD8, 0x20, 0x14, 0x23};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
//                            26 0D 20 AF
static const u4_t DEVADDR = 0x260D20AF; // <-- Change this address for every node!


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//------------------------------------------------------------------------------------------------------------------ PAYLOAD
// payload to send to TTN gateway
static uint8_t payload[13];
static osjob_t sendjob;
//------------------------------------------------------------------------------------------------------------------ TX TIME
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
// there is around a margin of time of 4 minutes plus to this time, so 120 (2 minutes) would be around 6 minutes of difference between one measure an the other, exactly 4 minutes and 16 seconds
const unsigned TX_INTERVAL = 60*5;// seconds  

// Pin mapping
//For TTGO LoRa32 V2.1.6 and TTGO T-Beam versions V0.x, V1.0 and V1.1:

const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 23,
  .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32}
};

//------------------------------------------------------------------------------------------------------------------ ON EVENT
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));

      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_RFU1:
      ||     Serial.println(F("EV_RFU1"));
      ||     break;
    */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    Serial.println(F("EV_SCAN_FOUND"));
      ||    break;
    */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}
//****************************************************************************************************************************  DO SEND
void do_send(osjob_t* j) {
  t_0 = millis();
  Serial.print("Time passed since the start of execution (start of do_send) [miliseconds]: ");
  Serial.println(t_0);
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {

    turnOffSystem(false); //the system is turn on
    waitPower();


    measurementRS485_Conductivity();
    measurementRS485_DO();
    voltInput = ((analogRead(batteryReadingPin)) * 3.3 / 4095.0) * 4.3; // da 11.45 sin la carga, da entre 10.67 y 10.75 con la carga(reles activados)
    printAverage();

    // adjust for the f2sflt16 range (-1 to 1)
    //This are not arrays, it's just the name
    float auxTempAverage = tempAverage / 100;
    float auxCondAverage = condAverage / 100;
    float auxSalinityAverage = salinityAverage / 100;
    float auxDOpAverage = (DOpAverage / 100) / 100;
    float auxDOpvAverage = DOpvAverage / 100;
    float auxVoltInput = voltInput / 100;


    // float -> int
    // note: this uses the sflt16 datum (https://github.com/mcci-catena/arduino-lmic#sflt16)

    // auxTempAverage
    uint16_t payloadTemp = LMIC_f2sflt16(auxTempAverage);
    // int -> bytes
    byte tempLow = lowByte(payloadTemp);
    byte tempHigh = highByte(payloadTemp);
    // place the bytes into the payload
    payload[0] = tempLow;
    payload[1] = tempHigh;

    // auxCondAverage
    uint16_t payloadCond = LMIC_f2sflt16(auxCondAverage);
    // int -> bytes
    byte condLow = lowByte(payloadCond);
    byte condHigh = highByte(payloadCond);
    payload[2] = condLow;
    payload[3] = condHigh;

    // auxSalinityAverage
    uint16_t payloadSalinity = LMIC_f2sflt16(auxSalinityAverage);
    // int -> bytes
    byte salinityLow = lowByte(payloadSalinity);
    byte salinityHigh = highByte(payloadSalinity);
    payload[4] = salinityLow;
    payload[5] = salinityHigh;

    // auxDOpAverage
    uint16_t payloadDOp = LMIC_f2sflt16(auxDOpAverage);
    // int -> bytes
    byte DOpLow = lowByte(payloadDOp);
    byte DOvHigh = highByte(payloadDOp);
    payload[6] = DOpLow;
    payload[7] = DOvHigh;

    // auxDOpvAverage
    uint16_t payloadDOpv = LMIC_f2sflt16(auxDOpvAverage);
    // int -> bytes
    byte payloadDOpvLow = lowByte(payloadDOpv);
    byte payloadDOpvtHigh = highByte(payloadDOpv);
    payload[8] = payloadDOpvLow;
    payload[9] = payloadDOpvtHigh;

    // auxVoltInput
    uint16_t payloadVoltInput = LMIC_f2sflt16(auxVoltInput);
    // int -> bytes
    byte voltInputLow = lowByte(payloadVoltInput);
    byte voltInputHigh = highByte(payloadVoltInput);
    payload[10] = voltInputLow;
    payload[11] = voltInputHigh;

    delay(2000);
    turnOffSystem(true);

    t_f = millis();
    Serial.print("Time passed since the start of execution (end of do_send) [miliseconds]: ");
    Serial.println(t_f);
    
    if (t_processInterval - (t_f - t_0) > 0){
      delay(t_processInterval - (t_f - t_0));
    }
    t_f2 = millis();
    Serial.print("Total time of the process [miliseconds]: ");
    Serial.println(t_f2 - t_0);

    // prepare upstream data transmission at the next possible time.
    // transmit on port 2 (the first parameter); you can use any value from 1 to 223 (others are reserved).
    // don't request an ack (the last parameter, if not zero, requests an ack from the network).
    // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.
    LMIC_setTxData2(port_numBoya, payload, sizeof(payload), 0);
    
    Serial.print("----------------------------------------------");
    Serial.print("---------------- END OF PROCESS --------------");
    Serial.println("----------------------------------------------");
  }
  // Next TX is scheduled after TX_COMPLETE event.
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void preTransmission() {           //Function for setting stste of Pins DE & RE of RS-485
  digitalWrite(MAX485_DE_RE, 1);
}
void postTransmission() {
  digitalWrite(MAX485_DE_RE, 0);
}
//********************************************************************************************************************************************** SETUP
void setupRS485() {

  //  initBoard(_u16BaudRate);
  // When the power is turned on, a delay is required.
  delay(1500);
  pinMode(MAX485_DE_RE, OUTPUT);
  pinMode(MAX485EnergyPin, OUTPUT);
  pinMode(sensorsEnergyPin, OUTPUT);

  digitalWrite(MAX485_DE_RE, LOW);
  digitalWrite(MAX485EnergyPin, HIGH);
  digitalWrite(sensorsEnergyPin, HIGH); // se activa con LOW y se apaga con HIGH, no sé por qué
  delay(5000);

  Serial.begin(_u16BaudRate);                                   //Baud Rate as 9600
  DOSensor.begin(idDOSensor, Serial);                           //Slave ID as idDOSensor
  conductivitySensor.begin(idConductivitySensor, Serial);     //Slave ID as idConductivitySensor

  conductivitySensor.preTransmission(preTransmission);         //Callback for configuring RS-485 Transreceiver correctly
  conductivitySensor.postTransmission(postTransmission);
  DOSensor.preTransmission(preTransmission);
  DOSensor.postTransmission(postTransmission);
}

//************************************************************************************************************************************** measurementRS485_Conductivity()
void measurementRS485_Conductivity() {
  static uint32_t iCond;
  uint8_t j;
  uint8_t resultCond;
  uint8_t resultWrite;

  iCond++;

  Serial.print("-------------- DATA READING CONDUCTIVITY SENSOR #");
  Serial.print(iCond);
  Serial.println(" --------------");
  for (int k = 0; k < totalNumMedidas; k++) {

    // set word 0 of TX buffer to least-significant word of LORACounter (bits 15..0)
    conductivitySensor.setTransmitBuffer(1, lowWord(iCond));

    // set word 1 of TX buffer to most-significant word of LORACounter (bits 31..16)
    conductivitySensor.setTransmitBuffer(0, highWord(iCond));

    // slave: write TX buffer to (2) 16-bit registers starting at register 0
    //resultWrite = conductivitySensor.writeMultipleRegisters(regSerialNumber[0], regSerialNumber[1]);

    // slave: read (6) 16-bit registers starting at register 2 to RX buffer
    resultCond = conductivitySensor.readHoldingRegisters(regTempCond[0], regTempCond[1]);
    delay(conducSensorMeasureDelay);

    uint8_t lenCond = regTempCond[2] & 0xff;//podría ser uint8_t?
    uint16_t dataCond[lenCond]; //podria meterle malloc

    Serial.print("count ");
    Serial.print(k + 1);
    Serial.println(" :");
    if (resultCond == conductivitySensor.ku8MBSuccess) {
      for (j = 0; j < lenCond / 2; j++) {
        dataCond[j] = conductivitySensor.getResponseBuffer(j);// & 0xff
        Serial.print(F(" dataCond["));
        Serial.print(j);
        Serial.print(F("]: "));
        Serial.print(dataCond[j], HEX);
      }
    }
    Serial.println();

    if (!dataToFloatCond(k, dataCond[1] & 0xff, dataCond[2] >> 8, dataCond[2] & 0xff, dataCond[3] >> 8,
                         dataCond[3] & 0xff, dataCond[4] >> 8, dataCond[4] & 0xff, dataCond[5] >> 8)) {

      k--;

    }
    memset(dataCond, 0x00, sizeof(dataCond));//Fill the array with zeros after ending
  }
  tempAverageCond = average(tempArrayCond, averageCorrectionCond);
  condAverage = average(condArray, averageCorrectionCond);
  salinityAverage = calculateSalinity(condAverage, tempAverageCond);
  tempAverage = tempAverageCond;// the temperature used in all the proyect is the temperature measured by the conductivity sensor
}


//******************************************************************************************************************************  DO MEASUREMENT
void measurementRS485_DO() {
  static uint32_t iDO;
  uint8_t j;
  uint8_t resultDO;
  uint8_t resultWrite;

  iDO++;

  Serial.print("-------------- DATA READING DO SENSOR #");
  Serial.print(iDO);
  Serial.println(" --------------");
  for (int k = 0; k < totalNumMedidas; k++) {

    // set word 0 of TX buffer to least-significant word of LORACounter (bits 15..0)
    DOSensor.setTransmitBuffer(1, lowWord(iDO));

    // set word 1 of TX buffer to most-significant word of LORACounter (bits 31..16)
    DOSensor.setTransmitBuffer(0, highWord(iDO));

    // slave: write TX buffer to (2) 16-bit registers starting at register 0
    resultWrite = DOSensor.writeMultipleRegisters(regSalinitySet[0], salinityAverage);

    // slave: read (6) 16-bit registers starting at register 2 to RX buffer
    resultDO = DOSensor.readHoldingRegisters(regTempDO[0], regTempDO[1]);
    delay(DOSensorMeasureDelay);

    Serial.print("Resultado escritura de salinidad");
    Serial.println(resultWrite);

    uint8_t lenDO = ceil(regTempDO[2] & 0xff);//podría ser uint8_t?
    uint16_t dataDO[lenDO]; //podria meterle malloc


    Serial.print("count ");
    Serial.print(k + 1);
    Serial.println(" :");
    if (resultDO == DOSensor.ku8MBSuccess) {
      for (j = 0; j < lenDO / 2; j++) {
        dataDO[j] = DOSensor.getResponseBuffer(j);// & 0xff
        Serial.print(F(" dataDO["));
        Serial.print(j);
        Serial.print(F("]: "));
        Serial.print(dataDO[j], HEX);
      }
    }
    Serial.println();

    if (!dataToFloatDO(k, dataDO[1] & 0xff, dataDO[2] >> 8, dataDO[2] & 0xff, dataDO[3] >> 8,
                       dataDO[3] & 0xff, dataDO[4] >> 8, dataDO[4] & 0xff, dataDO[5] >> 8,
                       dataDO[5] & 0xff, dataDO[6] >> 8, dataDO[6] & 0xff, dataDO[7] >> 8)) {

      k--;

    }
    memset(dataDO, 0x00, sizeof(dataDO));//Fill the array with zeros after ending
    delay(1000);
  }
  tempAverageDO = average(tempArrayDO, averageCorrectionDO);
  DOpAverage = average(DOpArray, averageCorrectionDO);
  DOvAverage = average(DOvArray, averageCorrectionDO);
  //tempAverage = (tempAverageDO + tempAverageCond) / 2;
  DOpvAverage = calculateDO(tempAverage, DOpAverage / 100, salinityAverage);// equation of datasheet
  //DOpvAverage = salinityCorrectionDO(DOp,tempAverageDO,salinityAverage);
}
//****************************************************************************************************************************  dataToFloatCond
bool dataToFloatCond(int k, uint8_t Temp0, uint8_t Temp1, uint8_t Temp2, uint8_t Temp3,
                     uint8_t Cond0, uint8_t Cond1, uint8_t Cond2, uint8_t Cond3) {
  float temp = bytesToFloat(Temp0, Temp1, Temp2, Temp3);// °C
  float Cond = bytesToFloat(Cond0, Cond1, Cond2, Cond3);// mS/cm

  if (verification(temp) && verification(Cond)) {
    tempArrayCond[k] = temp;
    condArray[k] = Cond;
    counterFalseReading = 0;

    doc["TemperaturaCond"] = String(tempArrayCond[k], 4);
    doc["Conductividad"] = String(condArray[k], 4);
    serializeJson(doc, Serial);
    Serial.println();
    delay(2500);
    doc.clear();
    return true;
  } else {
    Serial.println("- FALSE READING -");
    counterFalseReading++;
    if (counterFalseReading == 3) {
      tempArrayCond[k] = 0.0;
      condArray[k] = 0.0;
      averageCorrectionCond++;
      counterFalseReading = 0;
      return true;
    }
    return false;
  }
}
//****************************************************************************************************************************  dataToFloatDO
bool dataToFloatDO(int k, uint8_t Temp0, uint8_t Temp1, uint8_t Temp2, uint8_t Temp3,
                   uint8_t DOp0, uint8_t DOp1, uint8_t DOp2, uint8_t DOp3,
                   uint8_t DOv0, uint8_t DOv1, uint8_t DOv2, uint8_t DOv3) {

  float temp = bytesToFloat(Temp0, Temp1, Temp2, Temp3);// °C
  float DOp = bytesToFloat(DOp0, DOp1, DOp2, DOp3);// %
  float DOv = bytesToFloat(DOv0, DOv1, DOv2, DOv3);// mg/L

  if (verification(temp) && verification(DOp))  {
    tempArrayDO[k] = temp;
    DOpArray[k] = DOp * 100.0;
    DOvArray[k] = DOv;
    counterFalseReading = 0;

    doc["TemperaturaDO"] = String(tempArrayDO[k], 4);
    doc["DOpercentage"] = String(DOpArray[k], 4);
    serializeJson(doc, Serial);
    Serial.println();
    delay(2500);
    doc.clear();
    return true;
  } else {
    Serial.println("- FALSE READING -");
    counterFalseReading++;
    if (counterFalseReading == 3) {
      tempArrayDO[k] = 0.0;
      DOpArray[k] = 0.0;
      DOvArray[k] = 0.0;
      averageCorrectionDO++;
      counterFalseReading = 0;
      return true;
    }
    return false;
  }
}

//****************************************************************************************************************************  bytesToFloat
float bytesToFloat(long int byte3, long int byte2, long int byte1, long int byte0)
{
  long int realbyte0, realbyte1, realbyte2, realbyte3;
  char S;
  long int E, M;
  float D;
  realbyte0 = byte3;
  realbyte1 = byte2;
  realbyte2 = byte1;
  realbyte3 = byte0;
  if ((realbyte0 & 0x80) == 0) { //0x80=10000000, se toma primer byte
    S = 0; //Positive
  }
  else {
    S = 1; //Negative
  }
  E  = ((realbyte0 << 1) | (realbyte1 & 0x80) >> 7) - 127;
  M = ((realbyte1 & 0x7f) << 16) | (realbyte2 << 8) | realbyte3;
  D = pow(-1, S) * (1.0 + M / pow(2, 23)) * pow(2, E);
  return D;
}
//**************************************************************************************************************************** calculateDO
float calculateDO(float temp, float DOp, float salinity) { //DO(mg / L)
  float pressure = pressureDefault;
  float Phmg = (pressure * 760) / 101.325;
  //float t = 0.0;
  float T = 273.15 + temp;//t is current temperature which get from probe
  //1 kilogram of pure water equals 1 liter when reaching its maximum density of 1 kg/l, at the temperature of 39.2 °F or 4 °C
  float S = salinity;// grams per kg
  //float S = salinityDefault;// grams per kg

  float X1, X1_prima, u, u_prima, X2;
  X1_prima = (-173.4292) // X1’ = ln X1
             + 249.6339 * (100 / T)
             + 143.3483 * log(T / 100) //Function log() is equal to ln(x)
             + (-21.8492 * (T / 100))
             + S * (-0.033096 + (0.014259 * T) / 100
                    - 0.001700 * (T / 100) * (T / 100));
  X1 = exp(X1_prima);
  // log u = 8.10765 - (1750.286/ (235+t))
  u_prima = 8.10765 - (1750.286 / (235 + T)); // u’ = log u
  u = pow(10, u_prima); //u=10^u’
  Phmg = pressure * 760 / 101.325;
  X2 = ((Phmg - u) / (760 - u));
  return DOp * X1 * X2 * 1.4276;
}
//**************************************************************************************************************************** DO CORRECTION
float salinityCorrectionDO(float DOp, float temp, float S) {
  const float P = 1.0; // pressure at sea level
  float T = temp + 273.15; // kelvin
  float u = exp(11.8571 - (3840.70 / T) - (216961 / pow(T, 2))); //water vapor pressure
  float thetha_0 = 0.000975 - 0.000001426 * temp + 0.00000006436 * pow(temp, 2); // related to the second viral coeficient of oxigen
  float F_p = (P - u) * (1 - thetha_0 * P) / ((1 - u) * (1 - thetha_0)); // correctoin factor for pressure
  float F_s = exp(-S * (0.017674 - (10.754 / T) + (2140.7 / pow(T, 2)))); // correcton factor for salinity
  float DO_0 = exp(-139.34411 + (1.57579 * pow(10, 5) / T) - (6.642308 * pow(10, 7) / pow(T, 2)) + (1.243800 * pow(10, 10) / pow(T, 3)) - (8.621949 * pow(10, 11) / pow(T, 4)));
  float DO = DO_0 * F_s * F_p; //DO concentration in mg/L
  float DOcorregido = DOp * DO / 100; //
  return DOcorregido;
}
//**************************************************************************************************************************** AVERAGE
float average(float* floatArray, int correction) {
  float average = 0;
  for (int i = 0; i < totalNumMedidas; i++) {
    average += floatArray[i];
  }
  return average / (float(totalNumMedidas) - float(correction));
}

//**************************************************************************************************************************** PRINT AVERAGE
float printAverage() {
  Serial.println("------------------ AVERAGES ----------------");
  Serial.println("----------- Conductiviry sensor results ---------");
  Serial.print("Temperature of conductivity sensor average:");
  Serial.print(String(tempAverageCond, 4));
  Serial.println(" °C");
  Serial.print("Average of conductivity:");
  Serial.print(String(condAverage, 4));
  Serial.println(" mS/cm");
  Serial.print("Average of salinity:");
  Serial.print(String(salinityAverage, 4));
  Serial.println(" PPM");
  Serial.println("----------------- SENSOR DE DO ---------------");
  Serial.print("Temperature of DO sensor average:");
  Serial.print(String(tempAverageDO, 4));
  Serial.println(" °C");
  Serial.print("Average of percentage of DO with salinity cero:");
  Serial.print(String(DOpAverage, 4));
  Serial.println(" %");
  //  Serial.print("Average of DO measure by the sensor:");
  //  Serial.print(String(DOvAverage, 4));
  //  Serial.println(" mg/L");
  Serial.print("Average of DO concentration caculated based on the percentage of DO measured:");
  Serial.print(String(DOpvAverage, 4));
  Serial.println(" mg/L");
  Serial.println("----------------- OTHER MEASURES ---------------");
  Serial.print("Batery charge:");
  Serial.print(String(voltInput, 4));
  Serial.println(" V");
  //  Serial.print("Average between the Temperature of conductivity sensor average and the Temperature of DO sensor average:");
  //  Serial.print(String(tempAverage, 4));
  //  Serial.println(" °C");
}

//**************************************************************************************************************************** calculateSalinity
float calculateSalinity(float conductivity, float T) { //conductivity in milisimmens per centimeter
  float a0 = 0.008;
  float a1 = -0.1692 ;
  float a2 = 25.3851;
  float a3 = 14.0941;
  float a4 = -7.0261;
  float a5 = 2.7081;
  float b0 = 0.0005;
  float b1 = -0.0056;
  float b2 = -0.0066;
  float b3 = -0.0375;
  float b4 = 0.0636;
  float b5 = -0.0144;
  float k = 0.0162; // -2 °C < T < 35 °C

  float c0 = 0.676;
  float c1 = 0.02;
  float c2 = 0.00011;
  float c3 = -0.00000069698;
  float c4 = 0.000000001;

  float C_35 = 42.914; // C(35,15,0) = 42.914 mS*cm-1.
  float C_s = conductivity; // C(S,T,P) mS*cm-1.
  float r_T = c0 + c1 * T + c2 * pow(T, 2) + c3 * pow(T, 3) + c4 * pow(T, 4);

  float R_p = 1; // it´s assumed as 1
  float R = C_s / C_35;
  float R_T = R / (R_p * r_T);

  float S = a0 + a1 * sqrt(R_T) + a2 * R_T + a3 * sqrt(pow(R_T, 3)) + a4 * pow(R_T, 2) + a5 * sqrt(pow(R_T, 5)) + ((T - 15) / (1 + k * (T - 15))) * (b0 + b1 * sqrt(R_T) + b2 * R_T + b3 * sqrt(pow(R_T, 3)) + b4 * pow(R_T, 2) + b5 * sqrt(pow(R_T, 5)));

  return S; //returns in miligrams per liter (or miligrams per kg) which is equivalent to ppm
}
//****************************************************************************************************************************  turnOffSystem
void turnOffSystem(bool off) {
  if (off) {
    digitalWrite(MAX485EnergyPin, HIGH);
    digitalWrite(sensorsEnergyPin, HIGH);
  } else {
    digitalWrite(MAX485EnergyPin, LOW);
    digitalWrite(sensorsEnergyPin, LOW);
  }
}
//**************************************************************************************************************************** verification
bool verification(float val) {
  if (fabs(val) < 0.00001) {
    Serial.println(F("too small of a reading"));
    return false;
  } else {
    return true;
  }
}

//**************************************************************************************************************************** verification
void waitPower() {
  Serial.print(F("wait for "));
  Serial.print(conducSensorpowerOnDelay / 1000);
  Serial.println(F(" seconds"));
  Serial.print(F("POWER ON DEVICE"));
  for (int j = 0; j < conducSensorpowerOnDelay / 10000; j++) {
    Serial.print(F("."));
    delay(10000);
  }
  Serial.println(F("."));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  while (!Serial); // wait for Serial to be initialized
  setupRS485();
  delay(100);     // per sample code on RF_95 test
  Serial.println(F("*********************************** Starting ***********************************"));

  Serial.println(F("setup RS485"));



#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set. The LMIC doesn't let you change
  // the three basic settings, but we show them here.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915) || defined(CFG_au915)
  // NA-US and AU channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#elif defined(CFG_as923)
  // Set up the channels used in your country. Only two are defined by default,
  // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
  // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

  // ... extra definitions for channels 2..n here
#elif defined(CFG_kr920)
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#elif defined(CFG_in866)
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#else
# error Region not supported
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);
}

void loop() {


  os_runloop_once();;


}
