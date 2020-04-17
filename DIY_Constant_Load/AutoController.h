
#pragma once


#include "Helpers.h"

class AutoController
{
  private:
    RateLimiter updateGate;
    RateLimiter nextHalfCycle;

    int savedPWM;  // Saves the pwm setting during OFF times of the duty cycle

    // Parameters to control the load we draw from the battery, etc.

    int controlT;          // Are we trying for constant voltage, constant current, or constant power control?
    int shapeT;            // Is the load continuous, or squarewave
    double periodSecs;     // If it is not continuous, what is the period?
    double dutyCycle = 50; // And what is the duty cycle percentage (portion of the period under load)

    int terminationT;      // What stopping criteria are we using (none, or some of these below)
    double voltageThresh;  // Stop if source voltage falls below this
    double currentThresh;  // Stop if current draw falls below this
    double powerThresh;    // Stop if power draw falls below this
    double minutesToRun;   // Stop after this time

    double controllerTarget = 0;   // Target current/power set by the user via the rotary encoder

    enum RunState { Terminated, Paused, Active };

    enum ControlType { Amps, Watts };

    byte runState;

    long timeJobWasStarted; // in millis()



    // A sanity watchdog check. At a given PWM setting we know from the circuit what gate
    // voltage we expect on the MOSFET. From the MOSFET data sheet, and the voltage on the Drain
    // pin we have some idea of what current we ought to be seeing. If we are nowhere near that,
    // something is wrong.  In particular, the situation I want to prevent is when the current
    // source is externally limited / disconnected / faulty connection.  We don't want the
    // algorithm to just keeep cranking up the gate voltage, otherwise when we do
    // connect the battery we'll see fireworks.
    bool seemsSane(int pwm, double volts, double amps)
    {

      return true;   // TODO
      if (pwm >= 240 && amps < 0.6) return false;
      if (pwm >= 300 && amps < 1.3) return false;
      return true;
    }

    void Pause()
    {
      runState = Paused;
      nextHalfCycle.SetInterval((unsigned long) (periodSecs * 1000 / 100 * (100 - dutyCycle)));
      digitalWrite(13, LOW);
      savedPWM = pwmSetting;
      pwmSetting = 0;
      analogWrite16(powerControlPin, pwmSetting);
    }

    void Resume()
    {
      runState = Active;
      nextHalfCycle.SetInterval((unsigned long) (periodSecs * 1000 / 100 * dutyCycle));
      digitalWrite(13, HIGH);
      pwmSetting = savedPWM;
      analogWrite16(powerControlPin, pwmSetting);
    }

  public:



    AutoController()
    {
      updateGate.SetInterval(200);
      pinMode(13, OUTPUT);  // Built in LED on Arduino
    }

    void Setup()
    {
      terminationT = 0;
      minutesToRun = 0;
      voltageThresh = 0;
      currentThresh = 0;
      powerThresh = 0;
      periodSecs = 0;
      dutyCycle = 50;
      controlT = 0;
      shapeT = 0;
      controllerTarget = 0;

      int response = pickMenuItem(defaultMenu);
      if (response == 1) // NO = user wants to tweak some defaults
      {
        controlT = pickMenuItem(controlTypes);
        shapeT = pickMenuItem(shapes);
        periodSecs = 0;
        dutyCycle = 50;
        if (shapeT == 1) {
          periodSecs = InputViaRotaryReader(periodQuery, 10, 1, 30000, 1.0, "%s Secs", 0, 0);
          dutyCycle = InputViaRotaryReader(dutyQuery, 500, 1, 999, 0.1, "%s%%", 0, 1);
        }

        terminationT = pickMenuItem(termination);
        minutesToRun = 0;
        voltageThresh = 0;
        currentThresh = 0;
        powerThresh = 0;
        switch (terminationT) {
          case 0:
            break;
          case 1:  voltageThresh = InputViaRotaryReader(voltageCutoff, 100, 0, 30000, 0.1, "%sV", 0, 1);
            break;
          case 2: currentThresh = InputViaRotaryReader(currentCutoff, 100, 0, 30000, 0.01, "%sA", 0, 2);
            break;
          case 3: powerThresh = InputViaRotaryReader(powerCutoff, 1000, 0, 30000, 0.1, "%sW", 0, 1);
            break;
          case 4: minutesToRun = InputViaRotaryReader(timeLimit, 200, 0, 30000, 0.1, "%s minutes", 0, 1);
            break;
        }
        /*
              say("type", controlT);
              say("shape", shapeT);
              say("Period", periodSecs);
              say("Duty %", dutyCycle);
              say("VThresh", voltageThresh);
              say("IThresh", currentThresh);
              say("PThresh", powerThresh);
              say("mins", minutesToRun);
        */

      }
      lcd.ShowString(BL, ss[goPause]);
      while (encoderBtn.pollClick() == 0) {} // wait for down event
      runState = Active;

      if (shapeT > 0) { // intermittent load draw
        savedPWM = 0;
        Resume();
      }

      timeJobWasStarted = millis();
    }

    // Returns true to indicate we made changes, no further control is needed at this time.
    bool ToggleIntermittentDraw()
    {
      // If it is not an intermittent current draw, we're done.
      if (shapeT == 0) return false;

      // are we due to transition to to the other part of the squarewave cycle?
      if (!nextHalfCycle.hasGreenLight()) return false;

      // Flip state, save or restore old PWM setting, and set up future time for next flip
      if (runState == Paused) {
        Resume();
      }
      else {
        Pause();
      }
      return true;
    }

    void checkForTermination()
    {
      if (runState != Active) return;
      switch (terminationT) {
        case 0:  return;   // Manual control, never terminate automatically
          break;
        case 1: // voltageThresh = InputViaRotaryReader(voltageCutoff, 100, 0, 30000, 0.1, "%sV", 0, 1);
          break;
        case 2: //currentThresh = InputViaRotaryReader(currentCutoff, 100, 0, 30000, 0.01, "%sA", 0, 2);
          break;
        case 3:// powerThresh = InputViaRotaryReader(powerCutoff, 1000, 0, 30000, 0.1, "%sW", 0, 1);
          break;
        case 4:
          { unsigned long et = millis() - timeJobWasStarted;
            if (et >= minutesToRun * 60000) {
              runState = Terminated;
            }
          }
          break;
      }
    }

    void DoControlMoves()
    {
      // If cycle has already ended, its over.
      checkForTermination();
      if (runState == Terminated) return;
      if (ToggleIntermittentDraw()) return;
      // Now if we're in the off part of the cycle, don't exercise any control tweaks
      if (runState != Active) return;

      double shortfall = controllerTarget - amps;
      // But change our mind if it is power we are trying to control
      if (controlT == Watts) {
        shortfall = controllerTarget - watts;
      }

      double shortfallPcnt;
      if (controllerTarget == 0) {
        shortfallPcnt = 0;

        pwmSetting = 0;
        analogWrite16(powerControlPin, pwmSetting);
        // theController.SetPWM(0);
      }
      else {
        shortfallPcnt = (shortfall * 100.0) / controllerTarget;
      }

      double thresh = 4;

      int deltapwm = 0;

      if (shortfallPcnt > thresh)  {

        //  say("ShortfallPct is ", shortfallPcnt);

        // Don't allow the Gate to open any further if things look fishy.
        if (seemsSane(pwmSetting, volts, amps) && (temperatureC < 90)) {

          deltapwm = 1;
          if (shortfallPcnt > 30) {  // if we're far away speed up to get there quicker
            deltapwm = 5;
          }
          // In fact, cut the current to provide some overheating potection if we cross some upper threshold
          if (temperatureC >= 100) {
            deltapwm = -10;
          }
        }
      }
      else if (shortfallPcnt < -thresh)  {
        //  say("ShortfallPct is ", shortfallPcnt);
        deltapwm = -1;
        if (shortfallPcnt < -30) {
          deltapwm = -30; // panic a bit more, bring this down quickly
        }
      }

      pwmSetting += deltapwm;
      // sanity
      if (pwmSetting < 0) pwmSetting = 0;
      if (pwmSetting > 999) pwmSetting = 999; // Unchartered territory, we're bound to fry something.
      analogWrite16(powerControlPin, pwmSetting);
    }

    void ShowFeedback(bool isAuto) {

      // A bit convoluted because we're short of real estate.
      // So shift around fields, change number of decimal places, etc.

      showTemperature();

      lcd.ShowInt(Z02, "%3d", pwmSetting);

      if (controlT == Amps) {  // Constant current mode
        int numDecimals = 2;
        if (controllerTarget >= 10) {
          numDecimals = 1;
        }
        if (isAuto) {
          lcd.ShowDouble(Z00, "%sA", 4, numDecimals, controllerTarget);
        }
        else {
          lcd.ClearZone(Z00);
        }
        lcd.ShowDouble(Z10, "%sA", 4, numDecimals, amps);
      }
      else {                   // Constant load mode
        lcd.ShowDouble(Z00, "%sW", 4, 1, controllerTarget);
        lcd.ShowDouble(Z10, "%sW", 4, 1, watts);
      }

      lcd.ShowDouble(Z11, "%sV", 4, 1, volts);
      if (runState == Paused) {
        lcd.ShowString(Z12, "_\xF8_");
      }
      else if (runState == Terminated) {
        lcd.ShowString(Z12, "END");
      }
      else {
        if (controlT == Amps) {
          lcd.ShowDouble(Z12, "%sW", 4, 1, watts);
        }
        else {
          lcd.ShowDouble(Z12, "%sA", 4, 1, amps);
        }
      }
    }

    void UpdateTarget()
    {
      // Query whether the user has turned the knob to set the target
      int delta, posn;
      encoder.getVals(&delta, &posn);
      if (delta != 0) {
        if (controlT == Amps) {
          controllerTarget += 0.01 * delta;
        }
        else { // Controlling Watts
          controllerTarget += 0.1 * delta;
        }
        if (controllerTarget < 0) {
          controllerTarget = 0;
        }
      }
    }

    void MonitorAndTweak()
    {
      // Control the update frequency
      if (!updateGate.hasGreenLight()) return;

      UpdateTarget();
      DoControlMoves();
      ShowFeedback(true);
    }

    void ManualPWMControl()
    {
      // Control the update frequency
      if (!updateGate.hasGreenLight()) return;

      // In manual mode, force the logic to specific settings
      controlT = Amps;
      runState = Active;

      int delta, posn;   // Query whether the user has turned the knob
      encoder.getVals(&delta, &posn);

      if (delta != 0) {    // Exercise manual control over the PWM
        pwmSetting = posn;
        analogWrite16(powerControlPin, pwmSetting);
      }

      ShowFeedback(false);
    }
};
