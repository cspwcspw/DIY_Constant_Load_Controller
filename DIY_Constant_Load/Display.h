
#pragma once

#include <LiquidCrystal_I2C.h>

// An LCD display object derived from the standard library, but with 
// some extra support for format strings and printing into zones.
#define DispWidth 16
#define DispHeight 2

struct Zone { // Describe formatting for a small area of the display at (col, row).
  byte col;
  byte row;
  char *fmt;
};


// Pre-define some useful zones
Zone AL = {0, 0, "%-16s"};
//Zone AR = {0, 0, "%16s"};  // unused
Zone BL = {0, 1, "%-16s"};
Zone BR = {0, 1, "%16s"};
//Zone ER = {0, 1, "%6s"};  // unused
//Zone FR = {6, 1, "%8s"};   // unused

Zone Z00 = {0, 0, "%5s"};    // Zones on the top line
Zone Z01 = {5, 0, "%6s"};
Zone Z02 = {11, 0, "%5s"};
Zone Z03 = {5, 0, "%-11s"};  // spans  and Z02

Zone Z10 = {0, 1, "%5s"};    // Zones on lower line
Zone Z11 = {5, 1, "%6s"};
Zone Z12 = {11, 1, "%5s"};

//  D:\softworks\Arduino\libraries\LiquidCrystal_I2C/LiquidCrystal_I2C.h

// Some good LCD tips at https://www.baldengineer.com/arduino-lcd-display-tips.html
class ZonedDisplay : LiquidCrystal_I2C   {
   
  public:

    ZonedDisplay(byte width, byte height): LiquidCrystal_I2C(0x27, width, height) {}

    void myinit () {
      begin(42, 42);  // underlying library wants some dummy values
      init();
      backlight();
    }

    void ShowString(Zone z, char *str)
    {
      char buf[DispWidth + 1];
      sprintf(buf, z.fmt, str);
      setCursor(z.col, z.row);
      print(buf);
    }

    void ClearZone(Zone z)
    {
      ShowString(z, "");
    }

    void ShowInt(Zone z, const char *fmt, int val) {
      char buf2[DispWidth + 1];
      sprintf(buf2, fmt, val);
      ShowString(z, buf2);
    }

    void ShowLong(Zone z, const char *fmt, long val) {
      char buf2[DispWidth + 1];
      sprintf(buf2, fmt, val);
      ShowString(z, buf2);
    }

    void ShowDouble(Zone z, const char *fmt, int width, int precision, double val) {
      char buf2[DispWidth + 1];
      dtostrf(val, width, precision, buf2);
      char buf3[DispWidth + 1];
      sprintf(buf3, fmt, buf2);
      ShowString(z, buf3);
    }
};


ZonedDisplay lcd(DispWidth, DispHeight);  // A global singleton
