
// Pete Wentworth, March/April 2020

// Create Yet Another Constant Load Controller for testing batteries,
// solar panels, power supplies, etc.  Arduino UNO based.


#include "AsyncTemperatures.h"
#include "Helpers.h"
#include "AutoController.h"

// ESP32 Pins: 21=SDA;  22=SCL;   A0=VP=ADC1CH0   DAC1=25  DAC2=26

Settings mySettings; // Mainly for future perhaps, presently provides some calibration values

AutoController loadController;   // The logic to control the load




// The main program is always in one of a handful of modes
int mode = 0;
enum Modes { topLevelMenuMode, autoControlMode, manualPWMControlMode,
             lcdCharsetMode, playpenMode, numModes
           }; // numModes is a fake mode, tells us how many modes there are.

bool debug = false;




void setupMode(int newMode)  // Initialization code for each of the possible modes
{
  mode = newMode;
  //say("setupMode sets mode to ", mode);
  switch (mode) {

    case topLevelMenuMode:
      break;

    case playpenMode:
      lcd.ShowString(AL, ss[playPen]);
      lcd.ClearZone(BL);
      break;

    case manualPWMControlMode:
      lcd.ShowString(AL, ss[manualPWM]);
      lcd.ClearZone(BL);
      pwmSetting = 0;
      // theController.SetPWM(pwmSetting);
      analogWrite16(powerControlPin, pwmSetting);
      break;

    case lcdCharsetMode:
      lcd.ShowString(AL, ss[charset]);
      lcd.ClearZone(BL);
      pwmSetting = 0;
      break;

    case autoControlMode:
      loadController.Setup();  // Get control parameters from user
      break;
  }
}

void setup() {

  setupISR();

  Serial.begin(115200);
  Serial.print(ss[splash]); Serial.print(' '); Serial.println(ss[myVersion]);

  lcd.myinit();
  lcd.ShowString(AL, ss[splash]);
  lcd.ShowString(BL, ss[myVersion]);

  pinMode(voltageSensePin, INPUT);
  pinMode(currentSensePin, INPUT);

  setupPWM16();
  pinMode(voltageDoublerPWMPin, OUTPUT);
  analogWrite16(voltageDoublerPWMPin, topOfPWMRange / 2); // 50% duty cycle on pin 10

  // Start up the temperature sensing library
  myTemperatureSensors.begin();
  encoder.ResetValues(0, 0, 1024);
  delay(400);  // Leave the splash screen up for a moment...

  setupMode(0);  // Now start at top-level menu
}


void loop()
{
  static RateLimiter myGate(5);

  // We keep all the important measurements up do date no matter what mode we are in.

  if (myGate.hasGreenLight())
  {
    float historyWeight = mySettings.HistoryWeight;
    // Read and accumulate smoothed volts and amps over time
    volts = historyWeight * volts + (1.0 - historyWeight) * measureVoltage();
    amps = historyWeight * amps + (1.0 - historyWeight) * measureCurrent();
    // Its a bit ugly if we get small negative amp readings, so clamp them.
    if (amps < 0) amps = 0;
    watts = volts * amps;
  }

  occasionallyUpdateTemperature();

  processInputFromUser(); // While we are prototyping we allow some keyboard input too.

  switch (mode) {

    case topLevelMenuMode:  {
        int response = pickMenuItem(mainMenu);
        setupMode(response + 1);
      }
      break;

    case autoControlMode:
      //  showTemperature();
      loadController.MonitorAndTweak();
      break;

    case manualPWMControlMode:
      showTemperature();
      loadController.ManualPWMControl();
      break;

    case lcdCharsetMode: lcdCharSet();  break;
    case playpenMode: playpen();  break;
  }

  if (encoderBtn.pollClick())  { // exit whatever the current activity is.
    setupMode(0);
    pwmSetting = 0;
    analogWrite16(powerControlPin, pwmSetting);
  }
}

void processInputFromUser()
{
  // This software is currenty a lab playpen, so when there is a serial
  // port attached for debugging, it is useful infrastructure to let
  // the user interactively tweak things is useful.

  int n;
  if (Serial.available() > 0) {
    n = Serial.read();
    switch (n) {

      case 'm' : setupMode((mode + 1) % numModes);
        break;

      case 'w':
        {
          int16_t toBeSaved = (int16_t) encoder.getRotaryPosition();
          // sayf("\nTemp spike: saving rotaryPosition % d\n", toBeSaved);
          mySettings.Write16(0, toBeSaved);
        }
        break;

      case 'p' : Serial.println("Ping alive");  break;

      case 'd' : debug = ! debug;  break;
    }
  }
}


float measureVoltage()
{
  float resistorScaler = mySettings.VCalibrationCoarse + mySettings.VCalibrationFine / 1000.0;
  int v = analogRead(voltageSensePin);
  float myVolts = (v * resistorScaler) / 1024.0;
  //  Serial.print("A1 = "); Serial.print(v); Serial.print(" x scaler ("); Serial.print(resistorScaler);
  // Serial.print(") = ");  Serial.print(myVolts); Serial.print("  Global smoothed = "); Serial.println(volts);
  return myVolts;
}


float measureCurrent()
{
  // My current sensor is not very sensitive, and something is quite noisy.
  // It makes the control algorithm too twitchy, leading to instability.
  // So I'm sedating the patient quite aggressively, and adapting slowly.

  int numSmoothingReads = mySettings.ASmoothingReads;
  long ampVal = mySettings.ACalibrationFine;        // fine adjustment per batch readings
  long zeroCal =  mySettings.AZero;
  float currScaler = mySettings.AScaler;

  for (int i = 0; i < numSmoothingReads; i++) {
    ampVal += analogRead(currentSensePin) - zeroCal;
  }
  //  Serial.print(numSmoothingReads); Serial.print("x A readings sum = "); Serial.print(ampVal); Serial.print(" (scale "); Serial.print(currScaler*100); Serial.println("%)");
  double amps = (ampVal * currScaler) / (double) numSmoothingReads;
  return amps;
}

void lcdCharSet() {
  // Let the user scroll through the charset built into the LCD
  char buf[12];
  static int whichChar = 'A';
  int delta = encoder.getLastDelta();

  if (delta != 0) {
    whichChar = (whichChar + 256 + delta) % 256;
    sprintf(buf, "%3d 0x%02X %c", whichChar, whichChar, (char) whichChar);
    lcd.ShowString(Z03, buf);
  }
}

void playpen() {
  // Nothing that we're presently playing with.
}
