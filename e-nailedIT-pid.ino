
/////////////////////////////////////////////////////
// my e-nail Induction Tech controller,
// a PID closed loop temperature controller 
// for my ZVS induction coil e-nail heater.
// 
// released to the public domain  9/15/2016  hbf
//
////////////////////////////////////////////////////

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

int dabTemp = 450; // dab temp
int WindowSize = 1250; //  max output pulse width in msec
long purgeTime = 20000; // purge time in msec
unsigned long windowStartTime;
unsigned long runTime = 150000; // heat cycle run time in msec
unsigned long startTime; // heat cycle start time
boolean heating = false; // heat cycle running

void setup() {
  myOLED.begin();
  Serial.begin(230400);
  myPID.SetOutputLimits(0, WindowSize);
  setpoint = dabTemp; // init set point
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
     startTime = millis();
     setpoint = dabTemp + 50; // increase temp for purgeTime at start
     windowStartTime = millis();
     heating = true;
     myPID.SetMode(AUTOMATIC);  //turn the PID on
     led.write(HIGH); // turn on built-in led
   }
   if (heating) {
    if ((millis() - startTime) >= runTime) { // time's up, stop cycle
     heating = false; 
     myPID.SetMode(MANUAL);  //turn the PID off
     led.write(LOW);
    }
    if ((millis() - startTime) >= purgeTime) { // switch to dabTemp
     setpoint = dabTemp;
    }
    if (!isnan(tc) && (tc > 0)) {
     myPID.Compute();
     myOLED.print("*** heating ***", CENTER, 1);
     Serial.print(tc); // PID input
     Serial.print("  ");
     Serial.print(heater); // PID output
     Serial.print("  ");
     Serial.println(relay.read() * 300); // multiply relay state by an arbitrary number for plot
    }
   }

     myOLED.print("*F", RIGHT, 16);
     myOLED.print("sp", LEFT, 37);
     myOLED.printNumI(setpoint, CENTER, 37);
     myOLED.print("*F", RIGHT, 37);
     myOLED.print("sec", RIGHT, 48);
     myOLED.setFont(MediumNumbers);
     myOLED.printNumI(tc, CENTER, 16);
     if (heating) {
       myOLED.printNumI(((runTime - (millis() - startTime)) / 1000), CENTER, 48);
     }
     myOLED.update();
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

