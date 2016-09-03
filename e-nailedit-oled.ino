/////////////////////////////////////////////////////
// my e-nail controller,
// a simple closed loop temperature controller to 
// control a ZVS induction coil e-nail heater.
// 
// released to the public domain  8/18/2016  hbf
//
////////////////////////////////////////////////////

#include <OLED_I2C.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"

// create a thermocouple instance with software SPI on any three
// digital IO pins.
const int DO1 = 4;
const int CS1 = 5;
const int CLK1 = 6;
Adafruit_MAX31855 tc(CLK1, CS1, DO1);

//create OLED instance
extern uint8_t SmallFont[];
extern uint8_t MediumNumbers[];
OLED  myOLED(SDA, SCL, 8);

// define i/o, etc
const int relay = 7;
const int runpb = 8;
//const int cleanpb = 9;
//const int led = 13; // onboard led
int dabTemp = 420; // dab temp
//int cleanTemp = 1000; // clean temp
int setpoint = dabTemp; // init set point
// pulse the ZVS driver to allow the heat to soak through the work piece
int heattime = 800; // heat pulse on time in msec
int soaktime = 2200; // heat pulse off (soak) time in msec
//int lasttemp; // track temperature
unsigned long last; // last heat time
unsigned long dabtime = 100000; // heat cycle run time in msec
//unsigned long cleantime = 60000; // clean cycle run time in msec
unsigned long runtime = dabtime; // init run cycle time
unsigned long starttime; // heat cycle start time
boolean heating = false; // heat cycle running

void setup() {
  myOLED.begin();
  pinMode(runpb, INPUT);
  digitalWrite(runpb, HIGH);  // turn on internal pull-up resistor
//  pinMode(cleanpb, INPUT);
//  digitalWrite(cleanpb, HIGH);  // turn on internal pull-up resistor
  pinMode(relay, OUTPUT);
  digitalWrite(relay, LOW); //init relay to off
  //pinMode(led, OUTPUT);
  //digitalWrite(led, LOW); //init led to off
  Serial.begin(115200);
  Serial.println(setpoint);
}

void loop() {    
  int pv; // process variable
  
  myOLED.clrScr();
  myOLED.setFont(SmallFont);
 
  //print the current TC temp
   pv = tc.readFarenheit();
   Serial.println(pv);
   /*if (heating && (pv < (setpoint - 70)) && (pv <= (lasttemp + 30))) {
    heating = false; // fault condition, abort heating cycle    
   }*/
   if (heating && ((millis() - starttime) >= runtime)) { // time's up, stop cycle
    heating = false; 
    //setpoint = dabTemp;
    //runtime = dabtime;
   }
   if (digitalRead(runpb) == LOW && !heating) { // start cycle
    /*if (digitalRead(cleanpb) == LOW) { // change setpoint to cleaning temp
     setpoint = cleanTemp;
     runtime = cleantime;
    }*/
    starttime = millis();
    heating = true;
   }
   if (!isnan(pv) && (pv > 50) && heating) {
     if (pv < (setpoint - 2)) { // -2* helps to center deadband around setpoint
      myOLED.print("*** heat ***", CENTER, 1);
      long now = millis();
      if (now >= (last + soaktime)) {    
        digitalWrite(relay, HIGH);  // turn on relay
        //digitalWrite(led, HIGH);  // turn on led
        if (pv >= (setpoint - 50)) { // reduce heattime when within 50* of setpoint
          delay(heattime / 3.3);
        }
        else {
          delay(heattime); 
        }
        digitalWrite(relay, LOW);
        //digitalWrite(led, LOW);
        last = millis();
        //lasttemp = pv;
      }
     }
   }
     myOLED.print("*F", RIGHT, 16);
     myOLED.print("sp", LEFT, 37);
     myOLED.printNumI(setpoint, CENTER, 37);
     myOLED.print("*F", RIGHT, 37);
     myOLED.print("sec", RIGHT, 48);
     myOLED.setFont(MediumNumbers);
     myOLED.printNumI(pv, CENTER, 16);
     if (heating) {
       myOLED.printNumI(((runtime - (millis() - starttime)) / 1000), CENTER, 48);
     }
     myOLED.update();
   
   delay(200);
}


