
#pragma once

// Provide polled logic to query a button, debounce it, and 
// signal transitions.  Used here for the rotary encoder button.

class Button {

  private:
    int buttonPin;

    long buttonBlankedUntil = 0;     // for debouncing the switch
    int blankingMillisecs = 10;
    bool lastButtonState = true;     // For detecting switch transitions

  public:

    bool buttonUp = true;

    Button(int GPIO)
    {
      buttonPin = GPIO;
      pinMode(buttonPin, INPUT_PULLUP);      // Button is labelled SW on my device.
    }

    // Returns -1, 0 or 1 (down event, nothing happened, up event)
    int poll()
    {
      long timeNow = millis();
      if (timeNow >= buttonBlankedUntil)            // Are we outside any debounce blanking period
      {
        buttonUp = digitalRead(buttonPin);
        if (buttonUp != lastButtonState) {       // is there a transition?
          lastButtonState = buttonUp;
          buttonBlankedUntil = timeNow + blankingMillisecs;
          if (buttonUp) {
            return 1;
          }
          else {
             return -1;
          }
        }
      }
      return 0;
    }

    // Returns 0 (nothing happened) or 1 = click happened.  We busy-wait for the buttonUp.
    int pollClick()
    {
      int response = poll();
      if (response == 0) return 0;
      while (!buttonUp) {
         delay(1);
         response = poll();
      }
      return 1;
    }
};

const int buttonPin = 7;         // GPIO line
Button encoderBtn(buttonPin);    // The pushbutton on the rotary encoder
