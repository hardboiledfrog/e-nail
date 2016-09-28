
/////////////////////////////////////////////////////
// my e-nail Induction Tech controller,
// a PID closed loop temperature controller 
// for my ZVS induction coil e-nail heater.
// 
// released to the public domain  9/15/2016  hbf
//
////////////////////////////////////////////////////

#include <EEPROM.h>
#include <DirectIO.h>
#include <PID_v1.h>
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

// define PID variables
double setpoint, tc, heater;

// set up variables and initial tuning parameters
PID myPID(&tc, &heater, &setpoint,9,0.001,2, DIRECT);

// define i/o, etc
Input<8> runPb(true);
Output<7> relay(LOW);
Output<13> led(LOW);

int eepAddr = 0; // eeprom storage for dabCount
int dabCount;
int dabTemp = 440; // dab temp
int WindowSize = 1250; //  max output pulse width in msec
long purgeTime = 20000; // purge time in msec
unsigned long windowStartTime;
unsigned long runTime = 180000; // heat cycle run time in msec
unsigned long startTime; // heat cycle start time
unsigned long endTime; // heat cycle end time
boolean heating = false; // heat cycle running

void setup() {
  //EEPROM.write(eepAddr, 0); // clear stored dabCount
  myOLED.begin();
  Serial.begin(230400);
  myPID.SetOutputLimits(0, WindowSize);
  setpoint = dabTemp; // init set point
  dabCount = EEPROM.read(eepAddr); // init set point
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
// Timer Interrupt Handler
// ************************************************
ISR(TIMER1_COMPA_vect) {
  if (!heating)
  {
    relay.write(LOW);  // make sure relay is off
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
   if (heating) {
    if ((millis() - startTime) >= runTime || runPb.read() == LOW) { // time's up, stop cycle
      stopCycle();
    }
    if ((millis() - startTime) >= purgeTime) { // switch to dabTemp
     setpoint = dabTemp;
    }
    if (!isnan(tc) && (tc > 0)) { // if tc reading is valid
     myPID.Compute();
     myOLED.print("*** heating ***", CENTER, 1);
     logPID();
    }
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
      relay.write(HIGH);
     }
     else {
      relay.write(LOW);
     }
}

void startCycle() {
     startTime = millis();
     delay(500); // sketchy pb debounce
     if (startTime - endTime > 10000) {
       setpoint = dabTemp + 50; // increase temp for purgeTime at start
       dabCount ++;
       EEPROM.update(eepAddr,dabCount);
     } // else if within 10 seconds of last cycle stay at dabTemp
     heating = true;
     led.write(HIGH); // turn on built-in led
     myPID.SetMode(AUTOMATIC);  //turn the PID on
     windowStartTime = millis();
}

void stopCycle() {
     heating = false; 
     myPID.SetMode(MANUAL);  //turn the PID off
     led.write(LOW);
     endTime = millis();
     setpoint = dabTemp; // make sure we're displaying correct sp
     delay(500); // sketchy pb debounce
}

void logPID() {
     Serial.print(tc); // PID input
     Serial.print("  ");
     Serial.print(heater); // PID output
     Serial.print("  ");
     Serial.println(relay.read() * 300); // multiply relay state by an arbitrary number for plot
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

