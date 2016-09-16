/////////////////////////////////////////////////////
// my e-nail Induction Tech controller,
// a closed loop temperature controller sans PID 
// for my ZVS induction coil e-nail heater.
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
int relayState;
const int runpb = 8;
int dabTemp = 450; // dab temp
int setpoint = dabTemp; // init set point
int WindowSize = 1500; //  max output pulse width in msec
unsigned long windowStartTime;
long purgeTime = 20000; // purge time in msec
unsigned long runTime = 140000; // heat cycle run time in msec
unsigned long startTime; // heat cycle start time
boolean heating = false; // heat cycle running
double heater = 0; // relay on pulse width
int pv; // process variable

void setup() {
  myOLED.begin();
  pinMode(runpb, INPUT);
  digitalWrite(runpb, HIGH);  // turn on internal pull-up resistor
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // turn off led
  pinMode(relay, OUTPUT);
  digitalWrite(relay, LOW); //init relay to off
  Serial.begin(230400);
  windowStartTime = millis();

  // set up timer1 for 50ms interrupts 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  
  OCR1A  = 3125;            // compare match register 16MHz/256/20Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
ISR(TIMER1_COMPA_vect) {
  if (!heating)
  {
    digitalWrite(relay, LOW);  // make sure relay is off
  }
  else
  {
    driveOutput();
  }
}

void loop() {    
  
   myOLED.clrScr();
   myOLED.setFont(SmallFont);
   pv = tc.readFarenheit();
   if (!heating && digitalRead(runpb) == LOW) { // start cycle
    startTime = millis();
    setpoint = dabTemp + 50; // increase temp for purgeTime at start
    windowStartTime = millis();
    heating = true;
    digitalWrite(LED_BUILTIN, HIGH);
   }
   if (heating) { 
    myOLED.print("*** heating ***", CENTER, 1);
    Serial.print(pv); // for logging output
    Serial.print("  ");
    Serial.print(heater);
    Serial.print("  ");
    Serial.println(relayState * 300);
    if ((millis() - startTime) >= runTime) { // time's up, stop cycle
      heating = false; 
      digitalWrite(LED_BUILTIN, LOW);
    }
    if ((millis() - startTime) >= purgeTime) { // switch to dabTemp
     setpoint = dabTemp;
    }
    if (!isnan(pv) && (pv > 0) && (pv < setpoint)) {
     if (pv >= (setpoint - 15)) {
      // this heater's power is overkill so reduce output when within 50* of setpoint
      // for ramp up - 420 / 1500 * 100 = 28%
      // after setpoint 110 / 1500 * 100 = 7%  
       heater = 110;
     }
     else {
      heater = 420; 
     }
    }
    else {
     heater = 0;
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
     // display timer count down
     myOLED.printNumI(((runTime - (millis() - startTime)) / 1000), CENTER, 48);
   }
   myOLED.update();  
}

void driveOutput() {
       /************************************************
      * time proportion relay pin on/off based on heater value
      * Called by ISR every 50ms
      ************************************************/
     unsigned long now = millis();
     if(now - windowStartTime > WindowSize) { //time to shift the Relay Window
       windowStartTime += WindowSize;
     }
     if (heater > (now - windowStartTime)) {
      digitalWrite(relay,HIGH);
      relayState = 1; // for logging output
     }
     else {
      digitalWrite(relay,LOW);
      relayState = 0;
     }
}

