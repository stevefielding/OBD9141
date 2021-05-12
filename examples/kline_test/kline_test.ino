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
    
void loop(){
    Serial.println("Looping");
    digitalWrite(LED_BUILTIN, LOW);

    //bool init_success =  obd.init();
    bool init_success;
    // call init first time to get the state machine started
    obd.init(true, &init_success);
    // call repeatedly until init is done
    while (!obd.init(false, &init_success));

    //bool init_success =  obd.initKWP();
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("init_success:");
    Serial.println(init_success);
    
    // init_success = true;
    // Uncomment this line if you use the simulator to force the init to be
    // interpreted as successful. With an actual ECU; be sure that the init is 
    // succesful before trying to request PID's.

    if (init_success){
        bool reqSuccess = true;
        //bool res = true;
        while(reqSuccess){
          /*
            //res = obd.getCurrentPID(0x11, 1);
            obd.request9141_stMach(0x11, 0x01, 1);
            if (res){
                Serial.print("Result 0x11 (throttle): ");
                Serial.println(obd.readUint8());
            }
            */
            
            obd.request9141_stMach(true, &reqSuccess, 0x00, 0x01, 4);
            while (!obd.request9141_stMach(false, &reqSuccess, 0x00, 0x00, 0));
            if (reqSuccess){
                Serial.print("Result 0x00 (supported PIDs): ");
                Serial.println(obd.readUint32(), HEX);
            }
           /*
            //res = obd.getCurrentPID(0x0C, 0x01, 2);
            res = obd.request9141_stMach(0x0c, 0x01,  2);
            if (res){
                Serial.print("Result 0x0C (RPM): ");
                Serial.println(obd.readUint16()/4);
            }

            //res = obd.getCurrentPID(0x0D, 0x01, 1);
            res = obd.request9141_stMach(0x0d, 0x01, 1);
            if (res){
                Serial.print("Result 0x0D (speed): ");
                Serial.println(obd.readUint8());
            }

            res = obd.request9141_stMach(0x05, 0x01, 1);
            if (res){
                Serial.print("Result 0x05 (Coolant temp): ");
                Serial.println(obd.readUint8());
            }

            res = obd.request9141_stMach(0x00, 0x01, 4);
            if (res){
                Serial.print("Result 0xa0 (supported PIDs): ");
                Serial.println(obd.readUint32(), HEX);
            }
 
            res = obd.request9141_stMach(0xa6, 0x01, 4);
            if (res){
                Serial.print("Result 0xa6, (odometer): ");
                Serial.println(obd.readUint32(), HEX);
            }
            */
            Serial.println();

            delay(200);
        }
        delay(200);
    }
    delay(3000);
}
