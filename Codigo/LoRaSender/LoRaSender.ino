#include <LoRa.h>
#include "boards.h"
#include "W520.h"

// RS485 setup with TTGO T-beam
const int RE = 15;  // Connect RE terminal with 4 of TTGO T-beam
const int DE = 35;    // Connect DE terminal with 14 of TTGO T-beam
unsigned int t_15;
unsigned int t_35;
const byte ModReadBuffer[] = {0x01, 0x03, 0x09, 0x00, 0x00, 0x07, 0x07, 0x94}; //crc 0x494E
//byte BufferValue[8];
//SoftwareSerial mod(12, 34); // RX=12 , TX =34

W520 conductivitySensor;
byte ByteArray[250];
int alength;

const int baud = 9600;
const int serial = 115200;
const int Tx = 1;
int counter = 0;
int receivedInt = NULL;
bool salir = 0;
int b = 0;
String registroMain;


void setup()
{
  Serial.begin(serial);
  //mod.begin(serial);// modbus configuration
  conductivitySensor.config(&Serial, baud, SERIAL_8N1, Tx);
  conductivitySensor.setSlaveID(0x01);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  modbusTime(t_15, t_35, serial);

  initBoard();
  // When the power is turned on, a delay is required.
  delay(1500);
  //Serial.println("LoRa Sender");
  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);
  if (!LoRa.begin(LoRa_frequency)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop()
{

  alength = conductivitySensor.getTempCondRequest(DE, RE, ByteArray, t_15, t_35);

  String registros;
  for (b = 0 ; b < alength ; b++) {
    registros = String(ByteArray[b], HEX);
    registroMain+=registros;
  }
  //registroMain = funcionTemp();
  delay(2000);

#ifdef HAS_DISPLAY
  if (u8g2) {
    char buf[256];
    u8g2->clearBuffer();
    u8g2->drawStr(0, 12, "Temp and conduc");
    snprintf(buf, sizeof(buf), registroMain.c_str());
    u8g2->drawStr(0, 30, buf);
    u8g2->sendBuffer();
  }
#endif
}//cierre loop

int modbusTime(int t_15, int t_35, int serial) {
  if (serial > 19200) {
    t_15 = 750;
    t_35 = 1750;
  } else {
    t_15 = 15000000 / serial; // 1T * 1.5 = T1.5
    t_35 = 35000000 / serial; // 1T * 3.5 = T3.5
  }
}

void showMenu() {
  //Print main menu
  Serial.println("<MAIN MENU>");
  Serial.println("------------------------");
  Serial.println("1.)Serial Number");
  Serial.println("2.)getTempCondRequest");
  Serial.println("3.)OPTION 3");
  Serial.println("4.)Salir");
  Serial.println("------------------------");
}


void ejecutar(int receivedInt) {
  if (receivedInt != NULL) { //call SWITCH if input is available
    Serial.println("Entró en ejecutar");
    delay(100);
    switch (receivedInt) {
      case 1: //Serial number una vez

        break;
      case 2: //Temperatura y conductividad una vez


        break;
      case 3:
        Serial.println("Entró en case 3");
        delay(100);
        Serial.print("OPTION-");
        Serial.println(receivedInt);
        break;
      case 4:
        Serial.println("Entró en case 4");
        delay(100);
        Serial.print("OPTION-");
        Serial.println(receivedInt);
        salir = 1;
        break;
      default:
        Serial.println("Entró en case default");
        delay(100);
        Serial.println("!WRONG INPUT!");
        break;
    }
    receivedInt = NULL; //clear variable value
  }
}

//void funcionSerial() {
//
//  alength = conductivitySensor.getSerialNumberRequest(DE, RE, ByteArray, t_15, t_35);
//
//  char registros[alength];
//  // send packet
//  for (b = 0 ; b < alength ; b++) {
//    registros[b] = String(ByteArray[b], HEX);
//  }
//  String String(registros);
//  return registros;
//}
//
//String funcionTemp() {
//  
//  return registros;
//}
