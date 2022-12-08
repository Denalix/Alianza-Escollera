#include <LoRa.h>
#include "boards.h"
#include <ArduinoJson.h>
#include <ModbusMaster.h>               //Library for using ModbusMaster


// RS485 setup with TTGO T-beam
#define batteryReadingPin 2
#define MAX485_DE_RE  13  // Connect RE an DE terminals with pin 2 of TTGO T-beam   
#define MAX485EnergyPin 14
#define sensorsEnergyPin 25 //

const uint16_t _u16BaudRate = 9600;
const uint8_t idConductivitySensor = 2;
const uint8_t idDOSensor = 1;
const uint8_t conducSensorpowerOnDelay = 10000*2;//miliseconds, the multiplication is a security factor to assure that enough time has past
const uint8_t DOSensorpowerOnDelay = 4000*2;//miliseconds, , the multiplication is a security factor to assure that enough time has past
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
//int LORACounter = 0;
int counterFalseReading = 0;
const int totalNumMedidas = 5;

float tempArrayCond[totalNumMedidas];
float condArray[totalNumMedidas];
float tempArrayDO[totalNumMedidas];
float DOpArray[totalNumMedidas];//percentage array
float DOvArray[totalNumMedidas];//value array

float voltInput;

ModbusMaster conductivitySensor;                    //object conductivitySensor for class ModbusMaster
ModbusMaster DOSensor;                    //object conductivitySensor for class ModbusMaster
StaticJsonDocument<128> doc;

void preTransmission() {           //Function for setting stste of Pins DE & RE of RS-485
  digitalWrite(MAX485_DE_RE, 1);
}
void postTransmission() {
  digitalWrite(MAX485_DE_RE, 0);
}
//****************************************************************************************************************************  SETUP
void setup() {

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
  
  //turnOffSystem(false); //the system is turn on
  //(conducSensorpowerOnDelay >= DOSensorpowerOnDelay) ? delay(conducSensorpowerOnDelay) : delay(DOSensorpowerOnDelay); //Power On delay

  //delay(20000);//Power On delay
  //  Serial.println("LoRa Sender");
  //  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);
  //  if (!LoRa.begin(LoRa_frequency)) {
  //    Serial.println("Starting LoRa failed!");
  //    while (1);
  //  }
}
//****************************************************************************************************************************  LOOP
void loop() {
  turnOffSystem(false); //the system is turn on
  delay(conducSensorpowerOnDelay); //Power On delay
  voltInput = ((analogRead(batteryReadingPin))*3.3/4095.0)*4.24; // da 11.45 sin la carga, da entre 10.67 y 10.75 con la carga(reles activados)
  Serial.print("voltInput:");
  Serial.println(voltInput);
  delay(1000);
  Serial.println("ON");
  
  if (absoluteCount < totalNumMedidas) {
    
    Serial.print("count: ");
    Serial.print(absoluteCount);
    Serial.print(" - ");

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
    delay(1000);
    absoluteCount++;
  }
  if (absoluteCount == totalNumMedidas) {
    Serial.println("------------------ Promedios ----------------");
    Serial.println("----------- SENSOR DE CONDUCTIVIDAD ---------");
    Serial.print("Promedio de temperatura:");
    Serial.print(String(average(tempArrayCond), 4));
    Serial.println(" °C");
    //    loraPacket("tempCondAverage", String(average(tempArrayCond), 4));
    Serial.print("Promedio de conductividad:");
    Serial.print(String(calculateTDS(average(condArray)), 4));
    Serial.println(" mg/L");
    //    loraPacket("condAverage", String(calculateTDS(average(condArray)), 4));
    Serial.println("----------------- SENSOR DE DO ---------------");
    Serial.print("Promedio de temperaturaDO:");
    Serial.print(String(average(tempArrayDO), 4));
    Serial.println(" °C");
    Serial.print("Promedio de porcentaje de DO:");
    Serial.print(String(average(DOpArray), 4));
    Serial.println(" %");
    Serial.print("Promedio de DO medido de la sonda:");
    Serial.print(String(average(DOvArray), 4));
    Serial.println(" mg/L");
    Serial.print("Promedio de DO calculado del promedio de porcentaje de DO:");
    Serial.print(String(calculateDO(average(tempArrayDO), average(DOpArray) / 100), 4));
    Serial.println(" mg/L");
    Serial.println("---------------- Fin de proceso --------------");

    //    loraPacket("tempCondAverage", String(average(tempArrayCond), 4));
    delay(2000);
    turnOffSystem(true);
    absoluteCount++;
    Serial.print("Tiempo pasado desde inicio de ejecución en milisegundos: ");
    Serial.println(millis());
    delay(10000);
    absoluteCount = 0;
  }
  //delay(1800000 - millis());
}

//****************************************************************************************************************************  dataToJsonCond
void dataToJSONCond(uint8_t Temp0, uint8_t Temp1, uint8_t Temp2, uint8_t Temp3,
                    uint8_t Cond0, uint8_t Cond1, uint8_t Cond2, uint8_t Cond3) {

  float temp = bytesToFloat(Temp0, Temp1, Temp2, Temp3);// °C
  float Cond = bytesToFloat(Cond0, Cond1, Cond2, Cond3);// mS/cm

  if (verification(temp) && verification(Cond)) {
    tempArrayCond[absoluteCount] = temp;
    condArray[absoluteCount] = Cond;
    doc["TemperaturaCond"] = String(temp, 4);
    doc["Conductividad"] = String(Cond, 4);

    serializeJson(doc, Serial);
    Serial.println();
    delay(2500);

    //  loraPacket("TemperaturaCond", doc["TemperaturaCond"]);
    //  loraPacket("Conductividad", doc["Conductividad"]);

    doc.clear();
    counterFalseReading = 0;
  } else {
//    counterFalseReading++;
//    //absoluteCount--;
//    if (counterFalseReading >= 5) {
//      absoluteCount = totalNumMedidas;
//      Serial.println(F("Error en lectura de datos: más de 5 datos con valor menor a 0.00001"));
//    }
  }
}


void dataToJSONDO(uint8_t Temp0, uint8_t Temp1, uint8_t Temp2, uint8_t Temp3,
                  uint8_t DOp0, uint8_t DOp1, uint8_t DOp2, uint8_t DOp3,
                  uint8_t DOv0, uint8_t DOv1, uint8_t DOv2, uint8_t DOv3) {

  float temp = bytesToFloat(Temp0, Temp1, Temp2, Temp3);// °C
  float DOp = bytesToFloat(DOp0, DOp1, DOp2, DOp3);// %
  float DOv = bytesToFloat(DOv0, DOv1, DOv2, DOv3);// mg/L

  if (verification(temp) && verification(DOp) && verification(DOv))  {
    tempArrayDO[absoluteCount] = temp;
    DOpArray[absoluteCount] = DOp * 100.0;
    DOvArray[absoluteCount] = DOv;
      doc["TemperaturaDO"] = String(temp, 4);
      doc["PorcentajeOxigenoDisuelto"] = String(DOp, 4);
      doc["ValorOxigenoDisuelto"] = String(DOv, 4);

    //  JsonArray data = doc.createNestedArray("data");
    //  data.add(temp);
    //  data.add(Cond);

    // Serial.println("Testing JSON Format");
    serializeJson(doc, Serial);
    Serial.println();
    //  serializeJsonPretty(doc, Serial);

    //  Serial.print("Sending packet: ");
    //  Serial.println(LORACounter);
    //  delay(2500);

    //  loraPacket("TemperaturaDO", doc["TemperaturaDO"]);
    //  loraPacket("PorcentajeOxigenoDisuelto", doc["PorcentajeOxigenoDisuelto"]);
    //  loraPacket("ValorOxigenoDisuelto", doc["ValorOxigenoDisuelto"]);

    doc.clear();
    counterFalseReading = 0;
  } else {
//    counterFalseReading++;
//    //absoluteCount--;
//    if (counterFalseReading >= 5) {
//      absoluteCount = totalNumMedidas;
//      Serial.println(F("Error en lectura de datos: más de 5 datos con valor menor a 0.00001"));
//    }
//    if(absoluteCount < 0){
//      absoluteCount++;
//    }
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

//****************************************************************************************************************************  calculateDO
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
//****************************************************************************************************************************  AVERAGE
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
//****************************************************************************************************************************  ENERGY
void turnOffSystem(bool off) {
  if(off){
    digitalWrite(MAX485EnergyPin, HIGH);
    digitalWrite(sensorsEnergyPin, HIGH);
  } else{
    digitalWrite(MAX485EnergyPin, LOW);
    digitalWrite(sensorsEnergyPin, LOW);
  }
}
//****************************************************************************************************************************  VERIFICATION
bool verification(float val) {
  if (fabs(val) < 0.00001) {
    Serial.println(F("too small of a reading"));
    return false;
  } else {
    return true;
  }
}
//****************************************************************************************************************************  blinkPowerON


//void loraPacket(String var, String val) {
//  // send packet
//  LoRa.beginPacket();
//  LoRa.print(var);
//  LoRa.print(": ");
//  LoRa.print(val);
//  LoRa.print(" LORACounter: ");
//  LoRa.print(LORACounter);
//  LoRa.endPacket();
//  LORACounter++;
//  delay(5000);
//}
