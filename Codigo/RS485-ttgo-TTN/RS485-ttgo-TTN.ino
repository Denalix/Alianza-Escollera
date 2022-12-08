

 // References:
 // [feather] adafruit-feather-m0-radio-with-lora-module.pdf


#include <lmic.h>
#include <hal/hal.h>
#include <LoRa.h>
#include "boards.h"
#include <ArduinoJson.h>
#include <ModbusMaster.h>               //Library for using ModbusMaster

// RS485 setup with TTGO T-beam
#define MAX485_DE_RE  4  // Connect RE an DE terminals with pin 2 of TTGO T-beam   
#define energyControlPin 13


const uint16_t _u16BaudRate = 9600;
const uint8_t idConductivitySensor = 2;
const uint8_t idDOSensor = 1;
const uint8_t conducSensorpowerOnDelay = 10000;//miliseconds
const uint8_t DOSensorpowerOnDelay = 4000;//miliseconds
const uint8_t conducSensorMeasureDelay = 3000;//miliseconds
const uint8_t DOSensorMeasureDelay = 1000;//miliseconds

//Conductivity sensor registers
const uint16_t regSerialNumber[] = {0x0900, 0x0007, 0x0013};//startAddres|numberOfRegisters|numberOfResponseBytes
const uint16_t regTempCond[] = {0x2600, 0x0005, 0x000F};
//OD sensor registers
const uint16_t regTempDO[] = {0x2600, 0x0006, 0x0011};
const uint16_t regSalinitySet[] = {0x1500, 0x0002, 0x0011};//%. The default value is 0
const uint16_t regAtmPressureSet[] = {0x2400, 0x0002, 0x0008};//kPa. The default value is 101.325kpa
const uint16_t regSlaveIdGet[] = {0x3000, 0x0001, 0x0007};
const uint16_t regSlaveIdSet[] = {0x3000, 0x0001, 0x0008};
const uint16_t regCalibrationCoefficientsSet[] = {0x1100, 0x0004, 0x0008};
const uint16_t regCalibrationCoefficientsGet[] = {0x1100, 0x0004, 0x000D};

//OD default setting
const float pressureDefault = 101.325;// Kpa
const float salinityDefault = 0.0; // ppt or ‰
const float KDefault = 1.0; // Calibration coefficients
const float BDefault = 0.0;

int absoluteCount = 0;
int LORACounter = 0;
int counterFalseReading = 0;
const int totalNumMedidas = 15;

float tempArrayCond;
float condArray;
float tempArrayDO;
float DOpArray;//percentage array
float DOvArray;//value array

ModbusMaster conductivitySensor;                    //object conductivitySensor for class ModbusMaster
ModbusMaster DOSensor;                    //object conductivitySensor for class ModbusMaster
StaticJsonDocument<128> doc;
//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//


// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
//                                            AF F2 DE CF 26 15 CF A6 01 10 CB 68 CA E4 F0 63
static const PROGMEM u1_t NWKSKEY[16] = { 0xAF, 0xF2, 0xDE, 0xCF, 0x26, 0x15, 0xCF, 0xA6, 0x01, 0x10, 0xCB, 0x68, 0xCA, 0xE4, 0xF0, 0x63};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
//                                            07 18 C8 74 99 85 66 7E 30 C7 DC CD D8 20 14 23
static const u1_t PROGMEM APPSKEY[16] = { 0x07, 0x18, 0xC8, 0x74, 0x99, 0x85, 0x66, 0x7E, 0x30, 0xC7, 0xDC, 0xCD, 0xD8, 0x20, 0x14, 0x23};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260D20AF; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// payload to send to TTN gateway
static uint8_t payload[10];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
//For TTGO LoRa32 V2.1.6 and TTGO T-Beam versions V0.x, V1.0 and V1.1:

const lmic_pinmap lmic_pins = {
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32} 
};



void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        
        measurementRS485(); //all mearurements and data serial print is done here

        // adjust for the f2sflt16 range (-1 to 1)
        //This are not arrays, it's just the name
        tempArrayCond = tempArrayCond/100;
        condArray = condArray/100;
        
        tempArrayDO = tempArrayDO/100;
        DOpArray = DOpArray/100;
        DOvArray = DOvArray/100;
        
        
        // float -> int
        // note: this uses the sflt16 datum (https://github.com/mcci-catena/arduino-lmic#sflt16)
        uint16_t payloadTempCond = LMIC_f2sflt16(tempArrayCond);
        // int -> bytes
        byte tempCondLow = lowByte(payloadTempCond);
        byte tempCondHigh = highByte(payloadTempCond);
        // place the bytes into the payload
        payload[0] = tempCondLow;
        payload[1] = tempCondHigh;

        // float -> int
        uint16_t payloadCond = LMIC_f2sflt16(condArray);
        // int -> bytes
        byte condLow = lowByte(payloadCond);
        byte condHigh = highByte(payloadCond);
        payload[2] = condLow;
        payload[3] = condHigh;

        // float -> int
        uint16_t payloadTempDO = LMIC_f2sflt16(tempArrayDO);
        // int -> bytes
        byte TempDOLow = lowByte(payloadTempDO);
        byte TempDOHigh = highByte(payloadTempDO);
        payload[4] = TempDOLow;
        payload[5] = TempDOHigh;


        // float -> int
        uint16_t payloadDOp = LMIC_f2sflt16(DOpArray);
        // int -> bytes
        byte DOpLow = lowByte(payloadDOp);
        byte DOpHigh = highByte(payloadDOp);
        payload[6] = DOpLow;
        payload[7] = DOpHigh;

        // float -> int
        uint16_t payloadDOv = LMIC_f2sflt16(DOvArray);
        // int -> bytes
        byte DOvLow = lowByte(payloadDOv);
        byte DOvHigh = highByte(payloadDOv);
        payload[8] = DOvLow;
        payload[9] = DOvHigh;

        
        // prepare upstream data transmission at the next possible time.
        // transmit on port 2 (the first parameter); you can use any value from 1 to 223 (others are reserved).
        // don't request an ack (the last parameter, if not zero, requests an ack from the network).
        // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.
        LMIC_setTxData2(2, payload, sizeof(payload), 0);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

void preTransmission() {           //Function for setting stste of Pins DE & RE of RS-485
  digitalWrite(MAX485_DE_RE, 1);
}
void postTransmission() {
  digitalWrite(MAX485_DE_RE, 0);
}
void setupRS485() {

  //  initBoard(_u16BaudRate);
  // When the power is turned on, a delay is required.
  delay(1500);

  pinMode(MAX485_DE_RE, OUTPUT);
  pinMode(energyControlPin, OUTPUT);
  digitalWrite(MAX485_DE_RE, LOW);
  digitalWrite(energyControlPin, LOW);
  digitalWrite(energyControlPin, HIGH);
  Serial.begin(_u16BaudRate);                                   //Baud Rate as 9600

  DOSensor.begin(idDOSensor, Serial);                           //Slave ID as idDOSensor
  conductivitySensor.begin(idConductivitySensor, Serial);     //Slave ID as idConductivitySensor


  conductivitySensor.preTransmission(preTransmission);         //Callback for configuring RS-485 Transreceiver correctly
  conductivitySensor.postTransmission(postTransmission);
  DOSensor.preTransmission(preTransmission);
  DOSensor.postTransmission(postTransmission);

  (conducSensorpowerOnDelay >= DOSensorpowerOnDelay) ? delay(conducSensorpowerOnDelay) : delay(DOSensorpowerOnDelay); //Power On delay

}
void measurementRS485() {

  static uint32_t i;
  uint8_t j;
  uint8_t resultCond;
  uint8_t resultDO;

  i++;

  // set word 0 of TX buffer to least-significant word of LORACounter (bits 15..0)
  conductivitySensor.setTransmitBuffer(1, lowWord(i));
  DOSensor.setTransmitBuffer(1, lowWord(i));

  // set word 1 of TX buffer to most-significant word of LORACounter (bits 31..16)
  conductivitySensor.setTransmitBuffer(0, highWord(i));
  DOSensor.setTransmitBuffer(0, highWord(i));

  // slave: write TX buffer to (2) 16-bit registers starting at register 0
  //result = conductivitySensor.writeMultipleRegisters(regSerialNumber[0], regSerialNumber[1]);

  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  resultCond = conductivitySensor.readHoldingRegisters(regTempCond[0], regTempCond[1]);
  delay(conducSensorMeasureDelay);

  resultDO = DOSensor.readHoldingRegisters(regTempDO[0], regTempDO[1]);
  delay(DOSensorMeasureDelay);


  uint8_t lenCond = regTempCond[2] & 0xff;//podría ser uint8_t?
  uint8_t lenDO = ceil(regTempDO[2] & 0xff);//podría ser uint8_t?
  uint16_t dataCond[lenCond]; //podria meterle malloc
  uint16_t dataDO[lenDO]; //podria meterle malloc

  Serial.println("- Toma de datos: ");

  Serial.println(F("conductivity sensor response buffer: "));
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

  dataToJSONCond(dataCond[1] & 0xff, dataCond[2] >> 8, dataCond[2] & 0xff, dataCond[3] >> 8,
                 dataCond[3] & 0xff, dataCond[4] >> 8, dataCond[4] & 0xff, dataCond[5] >> 8);
  memset(dataCond, 0x00, sizeof(dataCond));//Fill the array with zeros after ending



  Serial.println(F("DO sensor response buffer: "));
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

  dataToJSONDO(dataDO[1] & 0xff, dataDO[2] >> 8, dataDO[2] & 0xff, dataDO[3] >> 8,
               dataDO[3] & 0xff, dataDO[4] >> 8, dataDO[4] & 0xff, dataDO[5] >> 8,
               dataDO[5] & 0xff, dataDO[6] >> 8, dataDO[6] & 0xff, dataDO[7] >> 8);
  memset(dataDO, 0x00, sizeof(dataDO));//Fill the array with zeros after ending

  //absoluteCount++;


  //delay(1800000 - millis());
}
void dataToJSONCond(uint8_t Temp0, uint8_t Temp1, uint8_t Temp2, uint8_t Temp3,
                    uint8_t Cond0, uint8_t Cond1, uint8_t Cond2, uint8_t Cond3) {

  float temp = bytesToFloat(Temp0, Temp1, Temp2, Temp3);// °C
  float Cond = bytesToFloat(Cond0, Cond1, Cond2, Cond3);// mS/cm

  //if (verification(temp) && verification(Cond)) {
    tempArrayCond = temp;
    condArray = Cond;
    doc["TemperaturaCond"] = String(temp, 4);
    doc["Conductividad"] = String(Cond, 4);

    serializeJson(doc, Serial);
    Serial.println();
    delay(2500);
    
    doc.clear();
    counterFalseReading = 0;
  //} else {
    counterFalseReading++;
    //absoluteCount--;
    //if (counterFalseReading >= 5) {
     // absoluteCount = totalNumMedidas;
      //Serial.println(F("Error en lectura de datos: más de 5 datos con valor menor a 0.00001"));
    //}
  //}
}


void dataToJSONDO(uint8_t Temp0, uint8_t Temp1, uint8_t Temp2, uint8_t Temp3,
                  uint8_t DOp0, uint8_t DOp1, uint8_t DOp2, uint8_t DOp3,
                  uint8_t DOv0, uint8_t DOv1, uint8_t DOv2, uint8_t DOv3) {

  float temp = bytesToFloat(Temp0, Temp1, Temp2, Temp3);// °C
  float DOp = bytesToFloat(DOp0, DOp1, DOp2, DOp3);// %
  float DOv = bytesToFloat(DOv0, DOv1, DOv2, DOv3);// mg/L

  //if (verification(temp) && verification(DOp) && verification(DOv))  {
    tempArrayDO = temp;
    DOpArray = DOp * 100.0;
    DOvArray = DOv;
    doc["TemperaturaDO"] = String(temp, 4);
    doc["PorcentajeOxigenoDisuelto"] = String(DOp, 4);
    doc["ValorOxigenoDisuelto"] = String(DOv, 4);

    // Serial.println("Testing JSON Format");
    serializeJson(doc, Serial);
    Serial.println();

    doc.clear();
    counterFalseReading = 0;
  //} else {
    counterFalseReading++;
   // absoluteCount--;
    //if (counterFalseReading >= 5) {
     // absoluteCount = totalNumMedidas;
      //Serial.println(F("Error en lectura de datos: más de 5 datos con valor menor a 0.00001"));
    //}
    //if (absoluteCount < 0) {
     // absoluteCount++;
    //}
  //}
}

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


float calculateDO(float t, float DOp) { //DO(mg / L)
  float pressure = pressureDefault;
  float Phmg = 0.0;
  //float t = 0.0;
  float T = 0.0;
  float S = 0.0;
  float X1, X1_prima, u, u_prima, X2;
  T = 273.15 + t; //t is current temperature which get from probe
  X1_prima = (-173.4292) // X1’ = ln X1
             + 249.6339 * (100 / T)
             + 143.3483 * log(T / 100) //Function log() is equal to ln(x)
             + (-21.8492 * (T / 100))
             + S * (-0.033096 + (0.014259 * T) / 100
                    - 0.001700 * (T / 100) * (T / 100));
  X1 = exp(X1_prima);
  // log u = 8.10765 - (1750.286/ (235+t))
  u_prima = 8.10765 - (1750.286 / (235 + t)); // u’ = log u
  u = pow(10, u_prima); //u=10^u’
  Phmg = pressure * 760 / 101.325;
  X2 = ((Phmg - u) / (760 - u));
  return DOp * X1 * X2 * 1.4276;
}

float average(float* floatArray) {
  float average = 0;
  for (int i = 0; i < totalNumMedidas; i++) {
    average += floatArray[i];
  }
  return average / float(totalNumMedidas);
}

float calculateTDS(float uScm) { //micro simmens per centimeter
  return uScm * 0.64; //returns in miligrams per liter which is equivalent to ppm
}

void energyControl(bool off) {
  (off) ? digitalWrite(energyControlPin, 0) : digitalWrite(energyControlPin, 1);
}

bool verification(float val) {
  if (fabs(val) < 0.00001) {
    Serial.println(F("too small of a reading"));
    return false;
  } else {
    return true;
  }
}
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

void setup() {
    
    while (!Serial); // wait for Serial to be initialized
    setupRS485();
    delay(100);     // per sample code on RF_95 test
    Serial.println(F("Starting"));

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
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    

    os_runloop_once();

}
