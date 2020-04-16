
#pragma once

// Retrieve some calibration settings from EEPROM, or
// save them to EEPROM.
// Not really used right now.  Most conversion factors are
// hard-wired into the code.

#include <EEPROM.h>

union twoBytes {
  uint8_t buf[2];
  uint16_t value1;
} val;

class Settings {

  public:

    int16_t Offset;

    int16_t VCalibrationCoarse;
    int16_t VCalibrationFine;

    int16_t ACalibrationCoarse;
    int16_t ACalibrationFine;
    int16_t AZero;

    int16_t ASmoothingReads;


    float AScaler;



    float HistoryWeight;

    int16_t Read16(int startAddr) {
      val.buf[0] = EEPROM.read(startAddr);
      val.buf[1] = EEPROM.read(startAddr + 1);
      return val.value1;
    }

    void Write16(int startAddr, int16_t theVal)
    {
      val.value1 = theVal;
      EEPROM.write(startAddr, val.buf[0]);
      EEPROM.write(startAddr + 1, val.buf[1]);
    }

    Settings() {   // read settings from EEPROM

      Offset = Read16(0);
      int16_t Offset;
      VCalibrationCoarse = 51;
      VCalibrationFine = 910;
      HistoryWeight = 0.96;
      ACalibrationFine = -16;
      AZero = 510;
      AScaler = 0.0697;
      ASmoothingReads = 100;
    }

};
