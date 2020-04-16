
#pragma once

// Read and interpret a rotary encoder.
// This must be polled, so here I you organize interrupts in the main loop to do so.

// Rotary devices use quadrature encoding with two digital signal lines.
// On this device, both signals are normally high when idle. On a rotation, both go
// low then high.  But rotation direction will determine whether
// the first line transitions before the other, or vice-versa.
// I put the two lines on adjacent pins on PORTD and use
// a port read to get their high/low states simultaneously.
// Treating the two bits as a binary number, I therefore read the value 0,1,2 or 3.
// Since the idle state value is 3, in theory there are only two possible
// sequences:  3102 or 3201. I see a lot of noise though, so using interrupts
// becomes quite messy.
// Instead have a small state machine that recognizes
// 310 or 320 transition sequences and filters out everything else.

// Rotary encoder is hardwired for the moment: ClkPin = 2; int DTPin = 3
// These are the only interrupt pins available on the UNO. (although I am not using interrupts).
// All on PORTD, with mask assumptions used in the code below.

// If the update is going to be called from an ISR we need to ensure that
// we disable interrupts while fetching variables from the class.


class RotaryEncoder {

  private:
    int clkPin = 4;
    int dtPin = 3;

    long lastRotaryEvent = 0;         // For tracking speed of rotation

    byte qVal, qValPrev;
    byte state = 3;

    int rotaryPosition;        // Accumulates the number of deltas up or down.
    int maxVal, minVal;        // Sets (inclusive) limits on rotaryPosition.

    int lastDelta;             // Changes seen on the encoder last time the updater recognized motion. 
                               
    int speedReward()     // Guess and scale the speed of rotation
    {
      static long lastRotaryEvent = 0;
      long timeNow = micros();
      long delta = timeNow - lastRotaryEvent;
      
      // Start with some guess for how fast the knob is turning
      int notchesPerSec = 500000l / delta;
      
      // Now make this weirdly non-linear to allow bigher jumps
      // for fast rotation. There are 3 bonus tiers.
      
      if (notchesPerSec >= 60) {   // Platinum - earn 6 times extra airmiles
        notchesPerSec *= 6;
      }
      else if (notchesPerSec >= 40) { // Gold - earn a bit less
        notchesPerSec *= 4;
      }
      else if (notchesPerSec >= 20) { // Silver - earn some bonus
        notchesPerSec *= 2;
      }

      // Some motion happened, so always give the user at least one notch.
      if (notchesPerSec == 0) notchesPerSec = 1;
      lastRotaryEvent = timeNow;
      return notchesPerSec;
    }

  public:

    RotaryEncoder()
    { // The theory says I shouldn't need pullups, but signals look a bit cleaner to me.
      pinMode(clkPin, INPUT_PULLUP);       // Rotary Encoder pin CLK on my device
      pinMode(dtPin, INPUT_PULLUP);        // Rotary encoder pin DT
      ResetValues(0, -1000, 1000);
    }

    void update()
    {
      const int READY = 3;
      const int CCW = 2;
      const int CW = 1;
      const int DETENTED = 0;

      // PIND reads DPort, bits correspond to pins 0-7
      // Isolate the two quadrature bits that we're interested in.
      qVal = (PIND & 0x0C) >> 2;
      if (qValPrev == qVal) return;  // No change since last poll
      qValPrev = qVal;

      // A small Finite State Machine driven by inputs from qVal.
      // State 3 is the ready state.
      // State 0 is the "ignore everything" state.
      // The debounce / signals are noisy on my cheap rotary encoder.
      // So I only accept input sequences 320... as a clockwise rotation.
      // And 310... is counter clockwise move.

      if (qVal == 3) {  // Both lines high. Synchronize,
        state = READY;      // or stay in the ready state
        return;
      }

      switch (state) {

        case DETENTED:  // We'll stay in this state, ignoring all noise until
          // logic above resets the state machine.
          break;

        case READY:  // Whichever line is low first sets the state to determine the direction of the move
          state = qVal;
          break;

        case CCW:
          if (qVal == 0) {  // Only when both lines are low do we accept the move
            state = DETENTED;
            lastDelta = -speedReward();         // scale for speed of rotation, and put in direction
            rotaryPosition += lastDelta;        // Add it to the overall position
            if (rotaryPosition < minVal) {      // Clamp it
              rotaryPosition = minVal;
            }
            break;

          case CW:
            if (qVal == 0) {  // Both lines low, accept a clockwise move
              state = DETENTED;
              lastDelta = speedReward();               
              rotaryPosition += lastDelta;
              if (rotaryPosition > maxVal) {
                rotaryPosition = maxVal;
              }
              break;
            }
          }
      }
    }

    void ResetValues(int InitialVal, int MinVal, int MaxVal)
    {
      // sayf("Encoder ResetValues %d  %d  %d  %d\n",InitialVal, MinVal,  MaxVal);
      noInterrupts();
      rotaryPosition = InitialVal;

      if (MinVal >= MaxVal) {         // sanity adjustment on limits
        MinVal = 0;
        MaxVal = 100;
      }
      maxVal = MaxVal;
      minVal = MinVal;

      // force rotaryPosition into range if we have to
      rotaryPosition = max(rotaryPosition, minVal);
      rotaryPosition = min(rotaryPosition, maxVal);

      interrupts();
    }


    // Returns two values
    // When this is called, lastDelta and lastClampedDelta reset to zero.
    void getVals(int *delta, int *posn)
    {
      noInterrupts();
      *delta = lastDelta;
      lastDelta = 0;
      *posn = rotaryPosition;
      interrupts();
    }

    // When this is called, lastDelta and lastClampedDelta reset to zero.
    uint32_t getLastDelta()
    {
      noInterrupts();
      int v = lastDelta;
      lastDelta = 0;
      interrupts();
      return v;
    }

    int getRotaryPosition()
    {
      noInterrupts();
      int v = rotaryPosition;
      interrupts();
      return v;
    }

};

RotaryEncoder encoder;  // Global singleton
