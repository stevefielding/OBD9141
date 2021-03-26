#include "Arduino.h"
#include "OBD9141.h"

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
    Serial.begin(9600);
    delay(2000);

    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH);

    obd.begin(Serial1, RX_PIN, TX_PIN);
    pinMode(LED_BUILTIN, OUTPUT);
}
    
void loop(){
    Serial.println("Looping");
    digitalWrite(LED_BUILTIN, LOW);

    bool init_success =  obd.init();
    //bool init_success =  obd.initKWP();
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("init_success:");
    Serial.println(init_success);
    
    // init_success = true;
    // Uncomment this line if you use the simulator to force the init to be
    // interpreted as successful. With an actual ECU; be sure that the init is 
    // succesful before trying to request PID's.

    if (init_success){
        bool res = true;
        while(res){
          /*
            res = obd.getCurrentPID(0x11, 1);
            if (res){
                Serial.print("Result 0x11 (throttle): ");
                Serial.println(obd.readUint8());
            }
            */
            res = obd.getCurrentPID(0x01, 4);
            if (res){
                Serial.print("Result 0x01 (supported PIDs): ");
                Serial.println(obd.readUint32());
            }
            
            res = obd.getCurrentPID(0x0C, 2);
            if (res){
                Serial.print("Result 0x0C (RPM): ");
                Serial.println(obd.readUint16()/4);
            }

/*
            res = obd.getCurrentPID(0x0D, 1);
            if (res){
                Serial.print("Result 0x0D (speed): ");
                Serial.println(obd.readUint8());
            }
            */
            Serial.println();

            delay(200);
        }
        delay(200);
    }
    delay(3000);
}
