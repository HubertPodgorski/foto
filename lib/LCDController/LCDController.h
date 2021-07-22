// LCDController.h
// Copyright (C) 2019 Alex Goris
// This file is part of FlyballETS-Software
// FlyballETS-Software is free software : you can redistribute it and / or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.If not, see <http://www.gnu.org/licenses/>

#ifndef _LCDCONTROLLER_h
#define _LCDCONTROLLER_h

#include "Arduino.h"
#include <LiquidCrystal_I2C.h>
class LCDControllerClass {
 protected:


 public:
	void init(LiquidCrystal_I2C* LCD1);
   void Main();
   enum LCDFields {
      D1Time,
      D1RerunInfo,
      D2Time,
      D2RerunInfo,
      D3Time,
      D3RerunInfo,
      D4Time,
      D4RerunInfo,
      D1CrossTime,
      D2CrossTime,
      D3CrossTime,
      D4CrossTime,
      RaceState,
      BattLevel,
      TeamTime,
      TotalCrossTime,
      BoxDirection
   };

   void UpdateField(LCDFields lcdfieldField, String NewValue);

private:
   void _UpdateLCD(int Line, int Position, String Text, int FieldLength);
   LiquidCrystal_I2C* _LCD1;
   unsigned long _LastLCDUpdate = 0;
   unsigned int _LCDUpdateInterval = 500; //500ms update interval

   struct SLCDField {
      int Line;
      int StartingPosition;
      int FieldLength;
      String Text;
   }_SLCDfieldFields[17];
};

extern LCDControllerClass LCDController;

#endif

