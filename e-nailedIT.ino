
/////////////////////////////////////////////////////
// my e-nail Induction Tech controller,
// a PID closed loop temperature controller 
// for my ZVS induction coil e-nail heater.
// 
// released to the public domain  9/15/2016  hbf
// a big thanks to the authors of the included libraries
////////////////////////////////////////////////////

#include <ArduPID.h>
#include <EEPROM.h>
#include <DirectIO.h>
#include <OLED_I2C.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"

// create a thermocouple instance with software SPI on any three
// digital IO pins.
const int DO1 = 4;
const int CS1 = 5;
const int CLK1 = 6;
Adafruit_MAX31855 thermocouple(CLK1, CS1, DO1);

// create OLED instance
extern uint8_t SmallFont[];
extern uint8_t MediumNumbers[];
OLED  myOLED(SDA, SCL, 8);

// define PID & variables
double WindowSize = 1250; //  max output pulse width in msec
double setpoint = 430;
double tc; // thermocouple PID input
double error;
double heater; // PID output
double Kp = 60; // tuning
double Ki = 5;
double Kd = 1;
double N = 100; //derivative filter constant D(s)=s/(1+s/N)
//a good rule is: N>10*Kd/Kp (also avoid too large values)
unsigned int period = 50; //50ms => 20Hz cycle frequency
unsigned int lastTime = 0;

PID_IC myPID(&heater, Kp, Ki, Kd, N, period); //PID with integrator clamping anti-windup

// define i/o, etc
Input<8> runPb(true); // set pin 8 for input & enable pull-up resistor
Output<7> relay(HIGH); // set pin 7 for output & relay off initial state (for BG relays LOW = on)
unsigned int eepDc = 0; // eeprom storage location for dabCount
unsigned int dabCount; // number of run cycles
unsigned long windowStartTime;
unsigned long runTime = 210000; // heat cycle run time in msec
unsigned long startTime; // heat cycle start time
unsigned long endTime; // heat cycle end time
boolean heating = false; // heat cycle running

void setup() {
  myOLED.begin();
  Serial.begin(230400);
  myPID.SetSaturation(0, WindowSize); //sets lower and upper limit to the PID output;
  //EEPROM.write(eepDc, 0); // clear or set stored dabCount
  dabCount = EEPROM.read(eepDc);
  windowStartTime = millis();

  // set up timer1 for 50ms interrupts 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 3125;            // compare match register 16MHz/256/20Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

// ************************************************
// Timer Interrupt Handler drives heater relay
// ************************************************
ISR(TIMER1_COMPA_vect) {
  if (!heating)
  {
    relay.write(HIGH);  // make sure relay is off
  }
  else
  {
    driveOutput();
  }
}

void loop() {      
   myOLED.clrScr();
   myOLED.setFont(SmallFont);
   tc = thermocouple.readFarenheit(); // read thermocouple
   if (!heating && runPb.read() == LOW) { // start cycle
    startCycle();
   }
   else if (heating) {
    runCycle();
   }
   displayData();
}

void driveOutput() {
       /************************************************
      * time proportion relay pin on/off based on pid output
      * called by ISR every 50ms
      ************************************************/
     unsigned long now = millis();
     if(now - windowStartTime > WindowSize) { //time shift the relay window
       windowStartTime += WindowSize;
     }
     if (heater > now - windowStartTime) {
      relay.write(LOW); // for BG relays LOW = on
     }
     else {
      relay.write(HIGH);
     }
}

void startCycle() {
     startTime = millis();
     delay(500); // sketchy pb debounce
     if (startTime - endTime > 10000) { // if within 10 seconds of last cycle don't increment counter
       dabCount ++;
       EEPROM.update(eepDc,dabCount);
     }
     heating = true;
     myPID.Reset();
     windowStartTime = millis();
}

void runCycle() {
    if ((millis() - startTime) >= runTime || runPb.read() == LOW) { // time's up, stop cycle
      stopCycle();
    }
    uint32_t now = micros();
    if ((now - lastTime) >= period) {
     lastTime = now;
     if (!isnan(tc) && (tc > 0)) { // if tc reading is valid
      error = setpoint-tc; //distance away from setpoint
      myPID.Compute(error);
     }
    }
    else if ((now - lastTime) < 0) {
     lastTime = 0L;
    }
    myOLED.print("*** heating ***", CENTER, 1);
    logPID();
}

void stopCycle() {
     heating = false; 
     endTime = millis();
     delay(500); // sketchy pb debounce
}

void logPID() {
     Serial.print(tc); // PID input
     Serial.print("  ");
     Serial.print(heater); // PID output
     Serial.print("  ");
     Serial.print(relay.read() * 300); // multiply relay state by an arbitrary number for plot
     Serial.print("  ");
     Serial.println(error); // PID error
}

void displayData() {
     myOLED.print("*F", 90, 16);
     myOLED.print("dc", LEFT, 37);
     myOLED.print("sp", 75, 37);
     myOLED.printNumI(dabCount, 30, 37);
     myOLED.printNumI(setpoint, 92, 37);
     myOLED.print("*F", RIGHT, 37);
     myOLED.print("sec", 92, 48);
     myOLED.setFont(MediumNumbers);
     myOLED.printNumI(tc, CENTER, 16);
     if (heating) {
       myOLED.printNumI(((runTime - (millis() - startTime)) / 1000), CENTER, 48);
     }
     myOLED.update();
}

