 
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
#include <RBD_Button.h>

// create a thermocouple instance with software SPI on any three
// digital IO pins.
const byte DO = 15;
const byte CS = 16;
const byte CLK = 17;
Adafruit_MAX31855 thermocouple(CLK, CS, DO);

// create OLED instance
// If using software SPI (the default case):
// check if correct display is used in header file
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
const byte OLED_CLK = 12;
const byte OLED_MOSI = 11;
const byte OLED_RESET = 10;
const byte OLED_DC = 9;
const byte OLED_CS = 8;
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
// declare OLED power pin
const byte v5d = 13;

// Rotary encoder declarations
const byte pinA = 2; // Our first hardware interrupt pin is digital pin 2
const byte pinB = 3; // Our second hardware interrupt pin is digital pin 3
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile int encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile int oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
// Menu and submenu/setting declarations
byte mode = 0;   // This is which menu mode we are in at any given time (top level or one of the submenus)
// Button reading
RBD::Button button(4); // this is the Arduino pin we are connecting the push button to

// define PID & variables
double windowSize = 1250; //  max output pulse width in msec
byte insert = 1; // flag for insert use
double setpoint;
double tc; // thermocouple PID input
double error; // = tc - setpoint 
double heater; // PID output
double Kp; // (60) PID tuning parameters 
double Ki; // (10)
double Kd; // (5)
double N; // (100) derivative filter constant D(s)=s/(1+s/N)
//a good rule is: N>10*Kd/Kp (also avoid too large values)
unsigned int period = 50U; //50ms => 20Hz cycle frequency

PID_IC myPID(&heater, Kp, Ki, Kd, N, period); //PID with integrator clamping anti-windup

// define i/o, etc
const byte spkr = 51; // speaker
const byte relay = 7; // output pin 7 drives relay which turns on ZVS heater module
volatile byte relayState;
unsigned int eepDc = 0; // eeprom storage location for dabCount
unsigned int eepSp = 2; // eeprom storage location for setpoint
unsigned int eepRut = 6; // eeprom storage location for runTime
unsigned int eepKp = 10; // eeprom storage location for PID tuning parameter
unsigned int eepKi = 14; // eeprom storage location for PID tuning parameter
unsigned int eepKd = 18; // eeprom storage location for PID tuning parameter
unsigned int eepN = 22; // eeprom storage location for PID tuning parameter
unsigned int eepIn = 26; // eeprom storage location for insert flag
unsigned int dabCount; // number of run cycles
unsigned long runTime = 510000; // heat cycle run time in msec
unsigned long windowStartTime;
unsigned long startTime; // heat cycle start time
unsigned long endTime; // heat cycle end time
unsigned long atSetpointTime; // setpoint achieved
boolean atSetpoint = false;
boolean heating = false; // heat cycle running

void setup() {
  pinMode(relay, OUTPUT); // heater control relay
  digitalWrite(relay, LOW); // init to off
  Serial.begin(230400);
  insert = EEPROM.readByte(eepIn);
  dabCount = EEPROM.readInt(eepDc);
  runTime = EEPROM.readLong(eepRut);
  setpoint = EEPROM.readDouble(eepSp);
  if (isnan(setpoint) || setpoint == 0) { // if EEPROM value bad load defaults
    setpoint = 500;
  }
  Kp = EEPROM.readDouble(eepKp);
  if (isnan(Kp) || Kp == 0) {
    Kp = 60;
  }
  Ki = EEPROM.readDouble(eepKi);
  if (isnan(Ki) || Ki == 0) {
    Ki = 10;
  }
  Kd = EEPROM.readDouble(eepKd);
  if (isnan(Kd) || Kd == 0) {
    Kd = 5;
  }
  N = EEPROM.readDouble(eepN);
  if (isnan(N) || N == 0) {
    N = 100;
  }
  myPID.SetTunings(Kp, Ki, Kd, N); //sets PID tuning parameters;
  myPID.SetSaturation(0, windowSize); //sets lower and upper limit to the PID output;
  //Rotary encoder section of setup
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(digitalPinToInterrupt(pinA), intPinA, RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(digitalPinToInterrupt(pinB), intPinB, RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  // set up menu variables for 1st pass
  encoderPos = 0; // reorientate the menu index - optional as we have overflow check code elsewhere
  oldEncPos = 10; // enables 1st "if" in rotaryMenu()
  mode = 0; // go back to top level of menu
  pinMode(v5d, OUTPUT); // setup pin as output to supply 5v to display
  digitalWrite(v5d, HIGH);
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
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
   if (heating) {
    runCycle();
   }
   else {
    unsigned long now = millis();
    if (now - endTime < 510000L && tc > 203 && tc < 205) { // reminder to clean bucket when cool enough
      tone(spkr, 500, 150);
      delay(500);
    }
    rotaryMenu();
   } 
}

void driveOutput() {
       /************************************************
      * time proportion relay pin on/off based on pid output
      * called by ISR every 50ms
      ************************************************/
     unsigned long now = millis();
     if(now - windowStartTime > windowSize) { //time shift the relay window
       windowStartTime += windowSize;
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
     if (insert) { // if using insert raise temp to heat soak dish
      setpoint = EEPROM.readDouble(eepSp) + 30;
     }
     else {
      setpoint = EEPROM.readDouble(eepSp);
     }
     dabCount ++;
     EEPROM.updateInt(eepDc,dabCount);
     myPID.Reset();
     heating = true;
     windowStartTime = millis();
}

void runCycle() {
static unsigned long lastTimePID = 0;
unsigned long now = millis();
    if ((now - startTime) >= runTime || button.onPressed()) { // time's up or knob pressed, stop cycle
      stopCycle();
      return;
    }
    if (!atSetpoint && tc >= setpoint) {
      atSetpoint = true;
      atSetpointTime = now;
    }
    else if (insert && now - atSetpointTime > 300000L && now - atSetpointTime < 301000L) { // heat soak insert for 6 min
     setpoint = EEPROM.readDouble(eepSp); // reduce setpoint to desired value
    }
    if(encoderPos != oldEncPos) { // adjust setpoint during run, +/- 5 degrees per detent 
      if(encoderPos > oldEncPos) {
        setpoint += 5;
      }
      else {
        setpoint -= 5;      
      }
      oldEncPos = encoderPos;
      setpoint = constrain(setpoint, 0, 650); // hard limits for setpoint
    }
    now = micros();
    if ((now - lastTimePID) >= period) { // compute PID at interval set by period
     lastTimePID = now;
     if (!isnan(tc) && (tc > 0)) { // if tc reading is valid
      error = setpoint-tc; //distance away from setpoint
      myPID.Compute(error);
     }
    }
    else if ((now - lastTimePID) < 0) {
     lastTimePID = 0;
    }
    displayRunData();
    logPID();
}

void stopCycle() {
     heating = false; 
     endTime = millis();
     atSetpoint = false;
     encoderPos = 0; // reorientate the menu index - optional as we have overflow check code elsewhere
     oldEncPos = 10; // enables 1st "if" in rotaryMenu()
     mode = 0; // go back to top level of menu
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
unsigned long now = millis();
     display.clearDisplay();
     display.setCursor(18,0);
     if (atSetpoint && insert && now - atSetpointTime > 360000L) { // wait until SiC insert reaches temp
      display.setTextColor(BLACK, WHITE); // display in black on white
      display.print("*** dab it! ***");
      display.setTextColor(WHITE);
      if (now - atSetpointTime < 361500L) { // beep when ready
       tone(spkr, 784, 200);
      }
     }
     else {
      display.print("*** heating ***");
     }
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
     display.print((runTime - (millis() - startTime)) / 1000L);
     display.display();
     display.setTextSize(1);
}

void rotaryMenu() { 
const byte modeMax = 8 * 5; // This is the number of submenus/settings you want * 5
// top menu section, check for knob turn, displays mode selections
  if(oldEncPos != encoderPos) {
    if (mode == 3) { // limit encoderPos to 0 or 1 for insert enable
      if (encoderPos < 0) {
        encoderPos = 1; // 
      }
      else if (encoderPos > 1) {
        encoderPos = 0;
      }
      oldEncPos = encoderPos;
    }
    display.setTextSize(2); // the encoder position prints but is immediately cleared if mode = 0
    display.setCursor(50,32);
    display.print(oldEncPos); //clears previous setting on display
    oldEncPos = encoderPos;
    display.setCursor(50,32);
    display.setTextColor(BLACK, WHITE); // display new setting in black on white
    display.print(encoderPos); // displays new setting when encoder rotated 
    display.setTextColor(WHITE);
    if (mode == 0) {
     int value;
     display.clearDisplay();
     display.setCursor(35,0);
     switch(encoderPos / 5) { 
      case 0: { // menu top, ready to run 
        break;
      }
      case 1: {
        display.println(" set");
        display.print("    temp");
        value = int(EEPROM.readDouble(eepSp)); // display current value
        break;
      }
      case 2: {
        display.println(" set");
        display.print("    time");
        value = EEPROM.readLong(eepRut) / 1000L; // display current value
        break;
      }
      case 3: {
        display.println(" set");
        display.print("  insert");
        value = insert; // display current value
        break;
      }
      case 4: {
        display.println(" set");
        display.print("    Kp");
        value = EEPROM.readDouble(eepKp); // display current value
        break;
      }
      case 5: {
        display.println(" set");
        display.print("    Ki");
        value = EEPROM.readDouble(eepKi); // display current value
        break;
      }
      case 6: {
        display.println(" set");
        display.print("    Kd");
        value = EEPROM.readDouble(eepKd); // display current value
        break;
      }
      case 7: {
        display.println(" set");
        display.print("    N");
        value = EEPROM.readDouble(eepN); // display current value
        break;
      }
      case 8: {
        display.println("clear");
        display.print("  counter");
        value = EEPROM.readInt(eepDc); // display current value
        break;
      }
     }
     if (encoderPos != 0) { // if on one of the setting screens
      display.setCursor(50,32);
      display.print(value); // display currently set value
     }
    }
  }

  //Main menu section, switches mode if knob pressed
  if (mode == 0) { // ready to run state, turn knob for settings, push knob to start / stop.
      if (encoderPos == 0) { // display ready screen
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(35,0);
        display.print("ready");
        unsigned long lastRun = (millis() - endTime) / 1000UL;
        int seconds = lastRun % 60UL;
        int minutes = lastRun / 60UL;       
        if (minutes < 10) { //display minutes:seconds from last run
          display.setCursor(46,22);
        }
        else if (minutes < 100) {
          display.setCursor(35,22);
        }
        else {
          display.setCursor(24,22);
        }
        display.print(minutes);
        display.print(":");
        if (seconds < 10) display.print("0");
        display.print(seconds);
        display.setTextSize(1);
        display.setCursor(86,48);
        display.print("*F");
        display.setCursor(46,48);
        display.setTextSize(2);
        display.print(int(tc)); // display temp when not running
        display.setTextSize(1);
      }
      if (encoderPos < 0) {
        encoderPos = modeMax; // check we haven't gone out of bounds below 0 and correct if we have
      }
      else if (encoderPos > modeMax) {
        encoderPos = 0; // check we haven't gone out of bounds above modeMax and correct if we have
      }
      if (button.onPressed()){ 
        mode = encoderPos / 5; // set the mode to the current value of encoder if knob pressed
        switch (mode) {
          case 0: { // start a heating cycle
            startCycle();
            break;
          }
          case 1: { // change temperature setpoint
            encoderPos = int(setpoint); // start adjusting temperature from last set point
            break;
          }
          case 2: { // change run time in seconds
            encoderPos = runTime / 1000; // start adjusting cycle run time from last set point
            break;
           }
          case 3: { // enable / disable insert function
            encoderPos = insert; // display current value
            break;
          }
          case 4: { // change PID tuning parameter
            encoderPos = Kp; // start adjusting Kp from last set point
            break;
          }
          case 5: { // change PID tuning parameter
            encoderPos = Ki; // start adjusting Ki from last set point
            break;
          }
          case 6: { // change PID tuning parameter
            encoderPos = Kd; // start adjusting Kd from last set point
            break;
          }
          case 7: { // change PID tuning parameter
            encoderPos = N; // start adjusting N from last set point
            break;
          }
          case 8: { // change dab counter total
            encoderPos = 0; // clear dab count
            break;
          }
        }
        if (mode != 0) { // if not starting, display current setting
          display.setCursor(50,32);
          display.setTextColor(BLACK, WHITE);
          display.print(encoderPos);
        }
      }
  }
  // if a change setting mode selected the next knob press changes the value
  else if (button.onPressed()) {
    switch (mode) {
      case 1: {
        setpoint = float(encoderPos); // record whatever encoder value to setting
        EEPROM.updateDouble(eepSp, setpoint); // only write eeprom if value changed
        break;
      }
      case 2: {
        runTime = long(encoderPos) * 1000UL;
        EEPROM.updateLong(eepRut, runTime);
        break;
      }
      case 3: {
        insert = encoderPos;
        EEPROM.updateByte(eepIn, insert);
        break;
      }
      case 4: {
        Kp = float(encoderPos);
        EEPROM.updateDouble(eepKp, Kp);
        break;
      }
      case 5: {
        Ki = float(encoderPos);
        EEPROM.updateDouble(eepKi, Ki);
        break;
      }
      case 6: {
        Kd = float(encoderPos);
        EEPROM.updateDouble(eepKd, Kd);
        break;
      }
      case 7: {
        N = float(encoderPos);
        EEPROM.updateDouble(eepN, N);
        break;
      }
      case 8: {
        dabCount = encoderPos;
        EEPROM.updateInt(eepDc, dabCount);
        break;
      }
    }
    if (mode >= 6 && mode <= 8) { // case 6 - 8
     myPID.SetTunings(Kp, Ki, Kd, N); //sets PID tuning parameters;
    }
    display.println("");
    display.setCursor(40,48);
    display.println("saved");
    display.display(); // update display
    delay(2000);
    encoderPos = 0; // reorientate the menu index - optional as we have overflow check code elsewhere
    oldEncPos = 10; // enables 1st "if" in rotaryMenu()
    mode = 0; // go back to top level of menu, now that we've set values
  }
  display.display(); // update display
}

//Rotary encoder interrupt service routine for one encoder pin
void intPinA(){
  noInterrupts(); //stop interrupts happening before we read pin values
  byte reading = PINE & 0x30; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00110000 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos -= 5; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00010000) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  interrupts(); //restart interrupts
}

//Rotary encoder interrupt service routine for the other encoder pin
void intPinB(){
  noInterrupts(); //stop interrupts happening before we read pin values
  byte reading = PINE & 0x30; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00110000 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos += 5; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00100000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  interrupts(); //restart interrupts
}


