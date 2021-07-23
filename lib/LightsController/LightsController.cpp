#include "LightsController.h"
#include "RaceHandler.h"

void LightsControllerClass::Init(
   uint8_t LIGHT_1_PIN,
   uint8_t LIGHT_2_PIN,
   uint8_t LIGHT_3_PIN,
   uint8_t LIGHT_4_PIN
) {
   _LIGHT_1_PIN = LIGHT_1_PIN;
   _LIGHT_2_PIN = LIGHT_2_PIN;
   _LIGHT_3_PIN = LIGHT_3_PIN;
   _LIGHT_4_PIN = LIGHT_4_PIN;
}

void LightsControllerClass::Main() {
   HandleStartSequence();

   //Check if we have to toggle any lights
   for (int i = 0; i < 4; i++) {
      if (millis() >= _LightsOnSchedule[i] && _LightsOnSchedule[i] != 0) {
         SetLightState(i, ON);
         _LightsOnSchedule[i] = 0; // Mark as schedule done
      }

      if (millis() >= _LightsOffSchedule[i] && _LightsOffSchedule[i] != 0) {
         SetLightState(i, OFF);
         _LightsOffSchedule[i] = 0; // Mark as schedule done
      }
   }
}

/// <summary>
///   Initiate start sequence, should be called if starting lights sequence should be initiated.
/// </summary>
void LightsControllerClass::InitiateStartSequence() {
   OverallState = STARTING;
}

/// <summary>
///   Deletes any scheduled light timings.
/// </summary>
void LightsControllerClass::DeleteSchedules() {
   //Delete any set schedules
   for (int lightIndex = 0; lightIndex < 4; lightIndex++) {
      _LightsOnSchedule[lightIndex] = 0;  //Delete schedule
      _LightsOffSchedule[lightIndex] = 0; //Delete schedule
   }
}

/// <summary>
///   Resets the lights (turn everything OFF).
/// </summary>
void LightsControllerClass::ResetLights() {
   OverallState = STOP;
   DeleteSchedules();
}

/// <summary>
///   Gets race state string. Internally the software uses a (enumerated) byte to keep the race
///   state, however on the display we have to display english text. This function returns the
///   correct english text for the current race state.
/// </summary>
///
/// <returns>
///   The race state string.
/// </returns>
String RaceHandlerClass::GetRaceStateString() {
   String strRaceState;
   
   switch (RaceState) {
      case RaceHandlerClass::STOP:
         strRaceState = " STOP";
         break;
      case RaceHandlerClass::STARTING:
         strRaceState = " STARTING";
         break;
      case RaceHandlerClass::RACING:
         strRaceState = "RACING";
         break;
      default:
         break;
   }

   return strRaceState;
}

/// <summary>
///   Handles the start sequence, will be called by main function when oceral race state is
///   STARTING.
/// </summary>
void LightsControllerClass::HandleStartSequence() {
   //This function takes care of the starting lights sequence
   //First check if the overall state of this class is 'STARTING'

   if (OverallState != STARTING) {
      return;
   }
   
   //The class is in the 'STARTING' state, check if the lights have been programmed yet
   if (!_StartSequenceStarted) { 
      //Start sequence is not yet started, we need to schedule the lights on/off times

      //Set schedule for RED light
      _LightsOnSchedule[0] = millis();         //Turn on NOW
      _LightsOffSchedule[0] = millis() + _msBetweenLights; //keep on for 1 second

      //Set schedule for YELLOW1 light
      _LightsOnSchedule[1] = millis() + _msBetweenLights;  //Turn on after 1 second
      _LightsOffSchedule[1] = millis() + (2 * _msBetweenLights); //Turn off after 2 seconds

      //Set schedule for YELLOW2 light
      _LightsOnSchedule[2] = millis() + (2 * _msBetweenLights);  //Turn on after 2 seconds
      _LightsOffSchedule[2] = millis() + (3 * _msBetweenLights); //Turn off after 3 seconds

      //Set schedule for GREEN light
      _LightsOnSchedule[3] = millis() + (3 * _msBetweenLights);  //Turn on after 3 seconds
      _LightsOffSchedule[3] = millis() + (4 * _msBetweenLights); //Turn off after 4 seconds

      _StartSequenceStarted = true;
   }
   
   //Check if the start sequence is busy
   bool StartSequenceBusy = false;
   for (int i = 0; i < 4; i++) {
      // check if any schedule is not yet done
      if (_LightsOnSchedule[i] > 0 || _LightsOffSchedule[i] > 0) {
         StartSequenceBusy = true;
      }
   }

   //Check if we should start the timer (GREEN light on and race is still in STARTING state)
   if (_LightsOnSchedule[3] == 0 && RaceHandler.RaceState == RaceHandler.STARTING) {
      RaceHandler.StartTimers();
      Serial.println("GREEN light is ON!");
   }

   if (!StartSequenceBusy) {
      _StartSequenceStarted = false;
      OverallState = RACING;
   }
}

/// <summary>
///   Set a given light to a given state.
/// </summary>
void LightsControllerClass::SetLightState(int LightIndex, LightStates LightState) {
   bool stateToSet = 0;

   if (LightState == ON) {
      stateToSet = 1;
   } else if (LightState == OFF) {
      stateToSet = 0;
   }

   if (LightIndex == 0) {
      digitalWrite(_LIGHT_1_PIN, stateToSet);
   } else if (LightIndex == 1) {
      digitalWrite(_LIGHT_2_PIN, stateToSet);
   }  else if (LightIndex == 2) {
      digitalWrite(_LIGHT_3_PIN, stateToSet);
   }  else if (LightIndex == 3) {
      digitalWrite(_LIGHT_4_PIN, stateToSet);
   }
}

LightsControllerClass LightsController;