#include "Arduino.h"
#include <OBD9141.h>

// In this example, Serial port 'Serial1' is used. The Serial port is used
// to provide information.

// The real Rx pin of the serial port, such that we can enable the pullup:
#define RX_PIN 19

// An extra pin connected to the Tx pin of the Serial port used. 
#define TX_PIN 18
// So this pin has a direct connection to pin 18 (TX1)

// The ENable pin for my MC33660 is connected to pin 22.
// Note that is pin22 following the Arduino Due pin numbering scheme
#define EN_PIN 28


OBD9141 obd;


void setup(){
    Serial.begin(115200);
    delay(2000);

    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH);

    obd.begin(Serial1, RX_PIN, TX_PIN);
    pinMode(LED_BUILTIN, OUTPUT);
}
    
enum klineStates { KLINE_INIT_START, KLINE_WAIT_INIT_DONE, KLINE_SUPP_PID,
                     KLINE_RPM, KLINE_THROT, KLINE_SPEED, KLINE_COOLANT_TEMP};
int klineCurrSt = KLINE_INIT_START;
bool init_success;
bool reqSuccess;

void loop(){
  //Serial.println("Looping");

  switch (klineCurrSt) {
    case KLINE_INIT_START:
      obd.init(true, &init_success);
      klineCurrSt = KLINE_WAIT_INIT_DONE;
      break;
    case KLINE_WAIT_INIT_DONE:
      if (obd.init(false, &init_success)) {
        if (init_success) {
          klineCurrSt = KLINE_SUPP_PID;
          obd.request9141_stMach(true, &reqSuccess, 0x00, 0x01, 4);
        }
        else
          klineCurrSt = KLINE_INIT_START;
      }
      break;
    case KLINE_SUPP_PID:
      if (obd.request9141_stMach(false, &reqSuccess, 0x00, 0x00, 0)) {
        if (reqSuccess){
          Serial.print("Supported PID: ");
          Serial.println(obd.readUint32(), HEX);
          obd.request9141_stMach(true, &reqSuccess, 0x0c, 0x01,  2);
          klineCurrSt = KLINE_RPM;
        }
        else {
          Serial.println("Failed to read supported PIDs");
          klineCurrSt = KLINE_INIT_START;
        }
      }
      break;
    case KLINE_RPM:
      if (obd.request9141_stMach(false, &reqSuccess, 0x00, 0x00, 0)) {
        if (reqSuccess){
          Serial.print("RPM: ");
          Serial.println(obd.readUint16()/4);
          obd.request9141_stMach(true, &reqSuccess, 0x11, 0x01,  1);
          klineCurrSt = KLINE_THROT;
        }
        else {
          Serial.println("Failed to read RPM");
          klineCurrSt = KLINE_INIT_START;
        }
      }
      break;
    case KLINE_THROT:
      if (obd.request9141_stMach(false, &reqSuccess, 0x00, 0x00, 0)) {
        if (reqSuccess){
          Serial.print("Throttle: ");
          Serial.println(obd.readUint8());
          obd.request9141_stMach(true, &reqSuccess,0x0d, 0x01,  1);
          klineCurrSt = KLINE_SPEED;
        }
        else {
          Serial.println("Failed to read Speed");
          klineCurrSt = KLINE_INIT_START;
        }
      }
      break;
    case KLINE_SPEED:
      if (obd.request9141_stMach(false, &reqSuccess, 0x00, 0x00, 0)) {
        if (reqSuccess){
          Serial.print("Speed: ");
          Serial.println(obd.readUint8());
          obd.request9141_stMach(true, &reqSuccess,0x05, 0x01,  1);
          klineCurrSt = KLINE_COOLANT_TEMP;
        }
        else {
          Serial.println("Failed to read Speed");
          klineCurrSt = KLINE_INIT_START;
        }
      }
      break;
    case KLINE_COOLANT_TEMP:
      if (obd.request9141_stMach(false, &reqSuccess, 0x00, 0x00, 0)) {
        if (reqSuccess){
          Serial.print("Coolant Temp: ");
          Serial.println(obd.readUint8());
          obd.request9141_stMach(true, &reqSuccess, 0x00, 0x01, 4);
          klineCurrSt = KLINE_SUPP_PID;
        }
        else {
          Serial.println("Failed to read Coolant temperature");
          klineCurrSt = KLINE_INIT_START;
        }
      }
      break;
    default:
      break;
  }
  //delay(200);
}
