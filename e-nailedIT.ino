
/////////////////////////////////////////////////////
// my e-nail Induction Tech controller,
// a PID closed loop temperature controller 
// for my ZVS induction coil e-nail heater.
// 
// released to the public domain  9/15/2016  hbf
// a big thanks to the authors of the included libraries
// and those below for the rotary menu code bits 
//*******Interrupt-based Rotary Encoder Menu Sketch*******
//  by Simon Merrett, based on insight from Oleg Mazurov, Nick Gammon, rt and Steve Spence, and code from Nick Gammon
///////////////////////////////////////////////////////

#include <EEPROMVar.h>
#include <EEPROMex.h>
#include <ArduPID.h>
#include <SPI.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// create a thermocouple instance with software SPI on any three
// digital IO pins.
const int DO = 15;
const int CS = 16;
const int CLK = 17;
Adafruit_MAX31855 thermocouple(CLK, CS, DO);

// create OLED instance
// If using software SPI (the default case):
#define OLED_CLK   12
#define OLED_MOSI  11
#define OLED_RESET 10
#define OLED_DC    9
#define OLED_CS    8
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
// declare OLED power pin
const byte v5d = 13;

// Rotary encoder declarations
const byte pinA = 2; // Our first hardware interrupt pin is digital pin 2
const byte pinB = 3; // Our second hardware interrupt pin is digital pin 3
const byte relay = 7;
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
unsigned int encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
unsigned int oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
// Button reading, including debounce without delay function declarations
const byte buttonPin = 4; // this is the Arduino pin we are connecting the push button to
const unsigned long debounceTime = 100UL;  // milliseconds
unsigned long buttonPressTime;  // when the switch last changed state
byte oldButtonState = HIGH;  // assume switch open because of pull-up resistor
boolean buttonPressed = 0; // a flag variable
// Menu and submenu/setting declarations
byte Mode = 0;   // This is which menu mode we are in at any given time (top level or one of the submenus)

// define PID & variables
double WindowSize = 1250; //  max output pulse width in msec
unsigned int eepSp = 2; // eeprom storage location for setpoint
double setpoint = 430;
double tc; // thermocouple PID input
double error;
double heater; // PID output
double Kp = 60; // tuning
double Ki = 5;
double Kd = 1;
double N = 100; //derivative filter constant D(s)=s/(1+s/N)
//a good rule is: N>10*Kd/Kp (also avoid too large values)
unsigned int period = 50U; //50ms => 20Hz cycle frequency
unsigned int lastTime = 0;

PID_IC myPID(&heater, Kp, Ki, Kd, N, period); //PID with integrator clamping anti-windup

// define i/o, etc
unsigned int eepDc = 0; // eeprom storage location for dabCount
unsigned int dabCount = 0; // number of run cycles
unsigned long windowStartTime;
unsigned int eepRt = 6; // eeprom storage location for runTime
unsigned long runTime = 210000UL; // heat cycle run time in msec
unsigned long startTime; // heat cycle start time
unsigned long endTime; // heat cycle end time
boolean heating = false; // heat cycle running
byte relayState;

void setup() {
  pinMode(relay, OUTPUT); // BG relay
  digitalWrite(relay, LOW); // relay module low true
  Serial.begin(230400);
  myPID.SetSaturation(0, WindowSize); //sets lower and upper limit to the PID output;
  // uncomment these lines to store initial values to new chip
  // be sure to comment them out when done or you'll burn out eeprom cells
  //EEPROM.updateDouble(eepSp, setpoint); // clear or set stored value
  //EEPROM.updateLong(eepRt, runTime); // clear or set stored value
  dabCount = EEPROM.readInt(eepDc);
  setpoint = EEPROM.readDouble(eepSp);
  runTime = EEPROM.readLong(eepRt);
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

  //Rotary encoder section of setup
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(digitalPinToInterrupt(pinA), intPinA, RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(digitalPinToInterrupt(pinB), intPinB, RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  // button section of setup
  pinMode(buttonPin, INPUT_PULLUP); // setup the button pin
  // set up menu variables for 1st pass
  encoderPos = 0; // reorientate the menu index - optional as we have overflow check code elsewhere
  oldEncPos = 10; // enables 1st "if" in rotaryMenu()
  buttonPressed = 0; // reset the button status so one press results in one action
  Mode = 0; // go back to top level of menu
  pinMode(v5d, OUTPUT); // setup pin as output to supply 5v to display
  digitalWrite(v5d, HIGH);
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
}

// ************************************************
// Timer Interrupt Handler drives heater relay
// ************************************************
ISR(TIMER1_COMPA_vect) {
  if (!heating)
  {
    digitalWrite(relay, LOW);  // make sure relay is off
    relayState = 0;
  }
  else
  {
    driveOutput();
  }
}

void loop() {      
   tc = thermocouple.readFarenheit(); // read thermocouple
  // Button reading with non-delay() debounce - thank you Nick Gammon!
  byte buttonState = digitalRead (buttonPin); 
  if (buttonState != oldButtonState){
    if (millis () - buttonPressTime >= debounceTime){ // debounce
      buttonPressTime = millis ();  // when we closed the switch 
      oldButtonState =  buttonState;  // remember for next time 
      if (buttonState == LOW){
        //Serial.println ("Button closed"); // DEBUGGING: print that button has been closed
        buttonPressed = 1;
      }
      else {
        //Serial.println ("Button opened"); // DEBUGGING: print that button has been opened
        buttonPressed = 0;  
      }  
    }  // end if debounce time up
  } // end of state change

   if (heating) {
    runCycle();
   }
   else {
    rotaryMenu();
   } 
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
      digitalWrite(relay, HIGH);
      relayState = 255;
     }
     else {
      digitalWrite(relay, LOW);
      relayState = 0;
     }
}

void startCycle() {
     startTime = millis();
     if (startTime - endTime > 10000) { // if within 10 seconds of last cycle don't increment counter
       dabCount ++;
       EEPROM.updateInt(eepDc,dabCount);
     }
     heating = true;
     myPID.Reset();
     windowStartTime = millis();
}

void runCycle() {
    if ((millis() - startTime) >= runTime || buttonPressed) { // time's up, stop cycle
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
    displayRunData();
    logPID();
}

void stopCycle() {
     heating = false; 
     endTime = millis();
     encoderPos = 0; // reorientate the menu index - optional as we have overflow check code elsewhere
     oldEncPos = 10; // enables 1st "if" in rotaryMenu()
     buttonPressed = 0; // reset the button status so one press results in one action
     Mode = 0; // go back to top level of menu
}

void logPID() {
     Serial.print(tc); // PID input
     Serial.print("  ");
     Serial.print(heater); // PID output
     Serial.print("  ");
     Serial.print(relayState); // relay state arbitrary number for plot
     Serial.print("  ");
     Serial.println(error); // PID error
}

void displayRunData() {
     display.clearDisplay();
     display.setCursor(18,0);
     display.print("*** heating ***");
     display.setCursor(90,16);
     display.print(" *F");
     display.setCursor(0, 37);
     display.print("dc ");
     display.print(dabCount);
     display.setCursor(70, 37);
     display.print("sp ");
     display.print(int(setpoint));
     display.print(" *F");
     display.setCursor(90, 48);
     display.print(" sec");
     display.setTextSize(2);
     display.setCursor(50,16);
     display.print(int(tc));
     display.setCursor(50, 48);
     display.print((runTime - (millis() - startTime)) / 1000);
     display.display();
     display.setTextSize(1);
}

void rotaryMenu() { 
const byte modeMax = 3; // This is the number of submenus/settings you want
// top menu sectio, check for knob turn, displays mode selections
  if(oldEncPos != encoderPos) {
    display.setTextSize(2); // the encoder position prints but is immediately cleared if Mode = 0
    display.setCursor(50,32);
//    display.setTextColor(WHITE, BLACK);
    display.print(oldEncPos); //clears previous setting on display
    display.setCursor(50,32);
    display.setTextColor(BLACK, WHITE); // display new setting in black on white
    display.print(encoderPos); // displays new setting when encoder rotated 
    display.setTextColor(WHITE);
    if (Mode == 0) {
     int value;
     display.clearDisplay();
     display.setCursor(35,0);
     switch(encoderPos) { 
      case 0: { // menu top, ready to run 
        display.print("ready");
        break;
      }
      case 1: {
        display.println(" set");
        display.print("    temp");
        value = (int(setpoint)); // display current value
        break;
      }
      case 2: {
        display.println(" set");
        display.print("    time");
        value = (runTime / 1000); // display current value
        break;
      }
      case 3: {
        display.println(" set");
        display.print("  counter");
        value = (dabCount); // display current value
        break;
      }
    }
    if (encoderPos != 0) { // if on one of the setting screens
     display.setCursor(50,32);
     display.print(value); // display currently set value
    }
    }
    oldEncPos = encoderPos;
  }

  //Main menu section, switches mode if knob pressed
  if (Mode == 0) { // ready to run state, turn knob for settings, push knob to start / stop.
      if (encoderPos == 0) { // display ready screen
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(35,0);
      display.print("ready");
      display.setTextSize(1);
      display.setCursor(90,40);
      display.print("*F");
      display.setCursor(50,40);
      display.setTextSize(2);
      display.print(int(tc)); // display temp when not running
      display.setTextSize(1);
      }
      if (encoderPos > (modeMax+10)) {
        encoderPos = modeMax; // check we haven't gone out of bounds below 0 and correct if we have
      }
      else if (encoderPos > modeMax) {
        encoderPos = 0; // check we haven't gone out of bounds above modeMax and correct if we have
      }
      if (buttonPressed){ 
        Mode = encoderPos; // set the Mode to the current value of encoder if knob pressed
        buttonPressed = 0; // reset the button status so one press results in one action
        switch (Mode) {
          case 0: { // start a heating cycle
            startCycle();
            break;
          }
          case 1: { // change temperature setpoint
            encoderPos = int(setpoint); // start adjusting temperature from last set point
            break;
          }
          case 2: { // change run time in seconds
            encoderPos = (runTime / 1000); // start adjusting cycle run time from last set point
            break;
           }
           case 3: { // change dab counter total
            encoderPos = dabCount; // start adjusting dab count from last set point
            break;
          }
        }
        display.setCursor(50,32);
        display.print(encoderPos);
       }
  }
  // if a change setting mode selected the next knob press changes the value
  else if (buttonPressed) {
    switch (Mode) {
      case 1: {
        setpoint = float(encoderPos); // record whatever encoder value to setting
        EEPROM.updateDouble(eepSp, setpoint); // only write eeprom if value changed
        break;
      }
      case 2: {
        runTime = long(encoderPos) * 1000UL;
        EEPROM.updateLong(eepRt, runTime);
        break;
      }
      case 3: {
        dabCount = encoderPos;
        EEPROM.updateInt(eepDc, dabCount);
        break;
      }
    }
    display.println("");
    display.setCursor(40,48);
    display.println("saved");
    display.display(); // update display
    delay(3000);
    encoderPos = 0; // reorientate the menu index - optional as we have overflow check code elsewhere
    oldEncPos = 10; // enables 1st "if" in rotaryMenu()
    buttonPressed = 0; // reset the button status so one press results in one action
    Mode = 0; // go back to top level of menu, now that we've set values
  }
  display.display(); // update display
}

//Rotary encoder interrupt service routine for one encoder pin
void intPinA(){
  noInterrupts(); //stop interrupts happening before we read pin values
  reading = PINE & 0x30; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00110000 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00010000) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  interrupts(); //restart interrupts
}

//Rotary encoder interrupt service routine for the other encoder pin
void intPinB(){
  noInterrupts(); //stop interrupts happening before we read pin values
  reading = PINE & 0x30; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00110000 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00100000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  interrupts(); //restart interrupts
}


