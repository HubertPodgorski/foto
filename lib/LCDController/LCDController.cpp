// file:	LCDController.cpp
//
// summary:	Implements the LCD controller class
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

#include "LCDController.h"
#include <LiquidCrystal_I2C.h>

/// <summary>
///   Initialises this object.
/// </summary>
///
/// <param name="LCD1">   [in,out] Pointer to first LCD object. </param>
void LCDControllerClass::init(LiquidCrystal_I2C *LCD1) {
   _LCD1 = LCD1;
   _LCD1->begin(40, 2);
   _LCD1->clear();

   //Put initial text on screen
   //                                 1         2         3
   //LCD layout:            0123456789012345678901234567890123456789
   _UpdateLCD(1, 0, String("1:   0.000s +  0.000s   | STOP   B:   0%"), 40);
   _UpdateLCD(2, 0, String("2:   0.000s +  0.000s   | Team:   0.000s"), 40);
   _UpdateLCD(3, 0, String("3:   0.000s +  0.000s   |   CR:   0.000s"), 40);
   _UpdateLCD(4, 0, String("4:   0.000s +  0.000s   |       Box: -->"), 40);

   _SLCDfieldFields[D1Time] = {1, 3, 7, String("  0.000")};
   _SLCDfieldFields[D1RerunInfo] = {1, 22, 2, String("  ")};
   _SLCDfieldFields[D2Time] = {2, 3, 7, String("  0.000")};
   _SLCDfieldFields[D2RerunInfo] = {2, 22, 2, String("  ")};
   _SLCDfieldFields[D3Time] = {3, 3, 7, String("  0.000")};
   _SLCDfieldFields[D3RerunInfo] = {3, 22, 2, String("  ")};
   _SLCDfieldFields[D4Time] = {4, 3, 7, String("  0.000")};
   _SLCDfieldFields[D4RerunInfo] = {4, 22, 2, String("  ")};
   _SLCDfieldFields[D1CrossTime] = {1, 12, 8, String("+  0.000")};
   _SLCDfieldFields[D2CrossTime] = {2, 12, 8, String("+  0.000")};
   _SLCDfieldFields[D3CrossTime] = {3, 12, 8, String("+  0.000")};
   _SLCDfieldFields[D4CrossTime] = {4, 12, 8, String("+  0.000")};
   _SLCDfieldFields[BattLevel] = {1, 36, 3, String("  0")};
   _SLCDfieldFields[RaceState] = {1, 25, 7, String(" STOP")};
   _SLCDfieldFields[TeamTime] = {2, 32, 7, String("  0.000")};
   _SLCDfieldFields[TotalCrossTime] = {3, 32, 7, String("  0.000")};
   _SLCDfieldFields[BoxDirection] = {4, 37, 3, String("-->")};
}

/// <summary>
///   Main entry-point for this application, this function should be called in every main loop
///   cycle. It will check whether the last time we updated the LCD screen is more than the given
///   timeout, and if yes, it will update the LCD screen with the latest data.
/// </summary>
void LCDControllerClass::Main() {
   //This is the main loop which handles LCD updates
   if ((millis() - _LastLCDUpdate) > _LCDUpdateInterval)
   {

      for (const SLCDField &lcdField : _SLCDfieldFields)
      {
         _UpdateLCD(lcdField.Line, lcdField.StartingPosition, lcdField.Text, lcdField.FieldLength);
      }

      _LastLCDUpdate = millis();
   }
}

/// <summary>
///   Updates a given pre-defined field on the LCD, with the new value.
/// </summary>
///
/// <param name="lcdfieldField"> The lcdfield identifier for which field should be updated </param>
/// <param name="NewValue">   The new value. </param>
void LCDControllerClass::UpdateField(LCDFields lcdfieldField, String NewValue) {
   if (_SLCDfieldFields[lcdfieldField].FieldLength < NewValue.length()) {
      //The new value will not fit into the new field!
      // TODO: do logging
      // ESP_LOGE(__FILE__, "[LCD Controller] Field (%i) received value that was too long (%i): %s", lcdfieldField, NewValue.length(), NewValue.c_str());
      return;
   }
   _SLCDfieldFields[lcdfieldField].Text = NewValue;
}

/// <summary>
///   Updates the LCD. This function will update the correct portion of the LCD, based on which line and position we want to update.
/// </summary>
///
/// <param name="Line">         Zero-based index of the line (1-4). </param>
/// <param name="Position">     Zero-based index of the starting position of the text which should be put on the screen. </param>
/// <param name="Text">       The text which should be put at the given position. </param>
/// <param name="FieldLength">  Length of the field, if the given text is longer than this value, the text will be made scrolling within the given field length. </param>
void LCDControllerClass::_UpdateLCD(int Line, int Position, String Text, int FieldLength) {
   //Check how long strMessage is:
   int MessageLength = Text.length();
   if (MessageLength > FieldLength) {
      //Message is too long, make it scroll!
      int ExtraChars = MessageLength - (FieldLength - 1);
      for (int i = 0; i < ExtraChars; i++) {
         String strMessageSubString = Text.substring(i, i + FieldLength);
         _LCD1->setCursor(Position, Line);
         _LCD1->print(strMessageSubString);
      }
      return;
   } else if (MessageLength < FieldLength) {
      //Message is too short, we need to pad it
      //First find missing characters
      int MissingChars = FieldLength - MessageLength;
      for (int i = 0; i < MissingChars; i++) {
         Text = String(Text + " ");
      }
   }
   _LCD1->setCursor(Position, Line);
   _LCD1->print(Text);
}

/// <summary>
///   The LCD controller.
/// </summary>
LCDControllerClass LCDController;