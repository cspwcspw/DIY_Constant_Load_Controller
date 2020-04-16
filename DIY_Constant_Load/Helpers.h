
#pragma once

// Pete Wentworth, March/April Lockdown, 2020

// Some pin assignments.
// In addition to the ones listed here, some pins are hardwired in the code:
// Pins A4(SDA) and A5(SCL) used by LiquidCrystal_I2C.h library to drive the LCD.
// Pins 2 (ClkPin) and 3 (DTpin) are dedicated to the rotary encoder.
// Pin 12 is used as the 1-wire bus to read the temperature sensor.
// Pin 13 is a debugPin - see AsyncTemperatures.h

// Using a square PWM pulse and a couple of diodes and capacitors we can "boost"
// our 5V to a higher voltage, which we need for a rail voltage for an OpAmp.
// https://www.youtube.com/watch?v=I4ED_8cuVTU

const int voltageDoublerPWMPin = 9;
const int powerControlPin = 10;
const int voltageSensePin = A1;
const int currentSensePin = A0;

const int coolingFanPin = 11;

#include "Settings.h"
#include "Display.h"
#include "RotaryEncoder.h"
#include "Button.h"

// Key long-term readings that we update and watch continuously in the main loop
float volts = 0;
float amps = 0;
float watts = 0;
float temperatureC = 16;
int pwmSetting = 0; // The variable that controls the gate voltage of the MOSFET.


// ************** Rate Limiter *****************************

// Utility for timing so that handlers can restrict 
// themselves to one event per green cycle. 
// A handler should only proceed if there is a green light.
// Getting a green automatically closes the gate again for
// a while. 

class RateLimiter {
  
  private:
    unsigned long lastGreen;
    unsigned long timeBetweenGreens;

  public:

    RateLimiter( ) { }
    
    RateLimiter(unsigned long _timeBetweenGreens)
    {
      SetInterval(_timeBetweenGreens);
    }
    
    void SetInterval(unsigned long _timeBetweenGreens)
    {
      timeBetweenGreens = _timeBetweenGreens;
      lastGreen = millis();
    }

    bool hasGreenLight() {
      unsigned long timeNow = millis();
      if ((unsigned long) (timeNow - lastGreen) >= timeBetweenGreens) {
        lastGreen = timeNow; 
        return true;
      }
      return false;
    }
};

// ********************** Temperature helpers ************************

bool hasNewTemperature;

void showTemperature()
{
  int numDecimals = 1;
  if (temperatureC >= 100) {
    numDecimals = 0;
  }
  lcd.ShowDouble(Z01, "%s\xDF", 4, numDecimals, temperatureC);  // The LCD character 0xDF is a degree symbol
}


// Sequences request to asynchronously convert temperature, and when
// that finishes, does steps to asynchronously read the temperature.
// Set hasNewTemperature true if a fresh temperature update has been done.

void occasionallyUpdateTemperature() {
  const float coolingFanOnThresh = 40;
  const float coolingFanOffThresh = 36;
  static bool fanIsON = false;

  static RateLimiter myGate(200); // 200 ms timeout is enough for any background temperature-related request.

  // Hard-wired ID for my temperature DS1820 physically near the power MOSFET
  static byte sensorID[] = {0x10, 0xEE, 0x4D, 0x26, 0x00, 0x08, 0x00, 0xCB};

  static byte scratchPad[9];  // buffer to hold temperature sensor data
  static byte state = 0;      // cycle through states 0,1,2,1,2,1,2...

  // In start state 0, Reset, Initiate conversion, transition to state 1;
  // In state 1, Check for completion (fail back to start state); Initiate reading of scratchpad, goto to state 2;
  // in state 2, Check for completion (fail back to start state); Convert to tempC, initiate next conversion, goto state 1;

  if (!myGate.hasGreenLight()) return;

   hasNewTemperature = false;
  // Poll the background async process to see if it has completed whatever it was doing.
  byte status = myTemperatureSensors.getStatus();
  if (status != 0) {
    Serial.print("TempC timeout = ");
    Serial.println(status);
    state = 0;  // reset back to start state
  }

  switch (state) {

    case 0: {
        myTemperatureSensors.convertAllTemperaturesAsync();
        state = 1;
      }
      return;

    case 1: {
        myTemperatureSensors.readScratchpadAsync(sensorID, scratchPad);
        state = 2;
      }
      return;

    case 2: {
        // Now we have a new temperature reading...
        temperatureC = myTemperatureSensors.getTempC(sensorID, scratchPad);
        myTemperatureSensors.convertAllTemperaturesAsync();
        state = 1;
      }
      break;
  }

  // We only get here if there is a new reading.

  if ((! fanIsON) && temperatureC >= coolingFanOnThresh) {
    fanIsON = true;
    pinMode(coolingFanPin, OUTPUT);
    digitalWrite(coolingFanPin, HIGH);
  }
  else if ((fanIsON) && temperatureC <= coolingFanOffThresh) {
    fanIsON = false;
    digitalWrite(coolingFanPin, LOW);
  }
  hasNewTemperature = true;
}

// *************** Periodic background interrupt ***********************
// This interrupt service routine is triggered by the built-in UNO 1ms timer.
// It can drive other occassionally polled logic.  Specifically here,
// we watch the rotary encoder by polling its lines rather than
// triggering and handling interrupts when the encoder is turned.

ISR(TIMER0_COMPA_vect)
{
  encoder.update();
}

void setupISR()
{
  // https://learn.adafruit.com/multi-tasking-the-arduino-part-2/timers
  // Timer0 on the Uno is already permanently counting and resetting, used
  // for millis() - we just put a new trigger point somewhere in the middle
  // of its counting cycle. When the count matches, it will call the "COMPA" ISR.
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}


// *************** String Store *********************************
// Strings might be eventually be externalized onto FLASH if we run out of 
// global space.  This indirection also potentially provides the basis for
// some easy localization or perhaps translation into other languages.

enum myStrings {
  splash, myVersion, constLoad, playPen, constCurrent, constPower,
  continuously, intermittently, never, lowVoltage, lowCurrent, lowPower,
  byTimeLimit, controlType, function, drawLoad, terminateOn,
  periodQuery, dutyQuery, voltageCutoff, currentCutoff, powerCutoff,
  timeLimit, goPause, defaultSettings, yes, no,
  charset, manualPWM
};

const char* store[] = {
  "Load Controller", "V0.2",
  "Const Load",  "Play pen",
  "Const Current I", "Const Power VI",
  "Continuously", "Intermittently",
  "never (manual)", "low voltage", "low current", "low power", "time limit",
  "Type of control?",
  "Function?", "Draw load?", "End load draw on",
  "Period on/off?",
  "Duty on-time?",
  "Voltage cutoff?",
  "Current cutoff?",
  "Power cutoff?",
  "Time limit?",
  "Click: Go/Pause",
  "Default Settings",
  "Yes",
  "No",
  "Chars",
  "Manual",
};

class StringStore {
  public:
    char *operator[](byte d) {
      return store[d];
    }
};

StringStore ss;


// *************** Menu Picker and Menu System *******************

// Menus are arrays of indexes into the string store.
// Every menu has a string store index to its title string, then the count of child items in the menu,
// followed by each of the child item indexes into the string store

byte mainMenu[] = {function, 4, constLoad, manualPWM, charset, playPen};
byte controlTypes[] = {controlType, 2, constCurrent, constPower};
byte shapes[] = {drawLoad, 2, continuously, intermittently};
byte termination[] = {terminateOn, 5, never, lowVoltage, lowCurrent, lowPower, byTimeLimit};
byte defaultMenu[] = {defaultSettings, 2, yes, no};

// Allows us to navigate and pick from one of these menus.
// Returns index of item picked, zero based.
int pickMenuItem(byte menu[]) {

  int selectedIndex = 0;
  int numItems = menu[1];
  lcd.ShowString(AL, ss[menu[0]]);
  lcd.ShowString(BL, ss[menu[selectedIndex + 2]]);

  while (true) {
    if (encoderBtn.pollClick() != 0) // button clicked
    {
      lcd.ClearZone(BL);
      return selectedIndex;
    }
    int delta = encoder.getLastDelta();
    if (delta > 0) {     // Move forward to the next item in the menu
      if (selectedIndex < numItems - 1) {
        selectedIndex++;
        lcd.ShowString(BL, ss[menu[selectedIndex + 2]]);
      }
    }
    else if (delta < 0) {     // Move back to the previous item in the menu
      if (selectedIndex > 0) {
        selectedIndex--;
        lcd.ShowString(BL, ss[menu[selectedIndex + 2]]);
      }
    }
  }
}

// *************** Input via Rotary Encoder **************

// Provide a gui-element kind of "input box" based on
// the rotary encoder, the LCD screen, and the rotary button

// Read, scale, clamp, and interpret the rotary encoder,
// showing the value onto the the LCD.  Collects a double value.
// This lets the user input timespans, or current settings, etc.
// User confirms the setting they want with a click.
// Like rest of the widgets in the project, it is intended to
// be setup, then polled/updated periodically.


double InputViaRotaryReader(byte titleIndx, int val, int min, int max, double scaler, const char *format, int width, int precision)
{
  double result = val * scaler;
  encoder.ResetValues(val, min, max);

  lcd.ShowString(AL, ss[titleIndx]);
  lcd.ShowDouble(BR, format, width, precision, result);

  while (true) {
    if (encoderBtn.pollClick() != 0) {
      return result;
    }
    // If we're not complete, has the user moved the dial?
    int delta, posn;
    encoder.getVals(&delta, &posn);
    if (delta != 0) {
      // Update the result and show it on the LCD.
      result = posn * scaler;
      lcd.ShowDouble(BR, format, width, precision, result);
    }
  }
}


// ************** Serial Output Helpers ********************

// Some diagnostic helpers for the Serial console
void say(char *hdr, int val)
{
  Serial.print(hdr); Serial.print(' '); Serial.println(val);
}

void say(char *hdr, double val)
{
  Serial.print(hdr); Serial.print(' '); Serial.println(val);
}


// ************** 12-bit PWM **************************

// Better than 8-bit PWM resolution needs some extra fiddles on timer2

const int numPWMbits = 10;
const unsigned int topOfPWMRange = 0xFFFF >> (16 - numPWMbits);

void setupPWM16() {

  //  https://www.sparkfun.com/datasheets/Components/SMD/ATMega328.pdf
  // Tradeoffs here. PWM works by repeatedly counting from zero up to TOP, then
  // resetting to zero.
  // Counting is driven by the (otionally scaled) CPU hardware clock.
  // At some comparator register match during the countup the output signal changes state.
  // If your top PWM value is smaller (fewer bits of PWM resolution)
  // the count-up happens quicker, so your PWM frequency increases.  So each time
  // we sacrifice 1 bit of resolution, we double the PWM frequency. Faster PWM
  // is easier to smooth / turn into a voltage with a low-pass filter RC circuit and
  // smaller capacitors, so the voltage following the PWM is more responsive.


  DDRB |= _BV(PB1) | _BV(PB2);        // set pins 9 and 10 as PWM outputs (part of DDRB)
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)  // Pg 135, we want non-inverting PWM outputs
           | _BV(WGM11);              // Pg 136, choose mode 14: fast PWM, TOP of range determined by ICR1
  TCCR1B = _BV(WGM13) | _BV(WGM12)    // Waveform generation
           | _BV(CS10) ;              // pg 137. Fastest we can get, hence no prescaling clock divider.

  ICR1 = topOfPWMRange;               // At 10-bit PWM, I get PWM carrier frequency about 15.6KHz

  /* https://arduino.stackexchange.com/questions/12718/increase-pwm-bit-resolution/12719
      Comments about the setup
    Changing ICR1 will effect the amount of bits of resolution.
    ICR1 = 0xffff; (65535) 16-bit resolution
    ICR1 = 0x7FFF; (32767) 15-bit resolution
    ICR1 = 0x3FFF; (16383) 14-bit resolution etc....

    Changing the prescaler bits in TCCR1B will effect the frequency of the PWM signal.
    Frequency[Hz}=CPU/(ICR1+1) where in this case CPU=16 MHz
    16-bit PWM will be>>> (16000000/8)/(65535+1)=30.5175Hz
  */
}

/* 16-bit version of analogWrite(). Works only on pins 9 and 10. */
void analogWrite16(int pin, int val)
{
  uint16_t uVal = (uint16_t) val;

  switch (pin) {
    case  9: OCR1A = uVal; break;
    case 10: OCR1B = uVal; break;
  }
}

 
// ************************************************
