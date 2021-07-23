#ifndef _LIGHTSCONTROLLER_h
#define _LIGHTSCONTROLLER_h

#include "Arduino.h"
#include "Structs.h"

class LightsControllerClass {
   public:
      void Main();
      void Init(uint8_t LIGHT_1_PIN,  uint8_t LIGHT_2_PIN, uint8_t LIGHT_3_PIN, uint8_t LIGHT_4_PIN);
      void HandleStartSequence();

      enum LightStates {
         OFF,
         ON
      };
      LightStates CheckLightState(int LightIndex);
      void SetLightState(int LightIndex, LightStates LightState);

      void InitiateStartSequence();
      
      enum OverallStates {
         STARTING,
         RACING,
         STOP
      };
      OverallStates OverallState = STOP;

      void DeleteSchedules();
      void ResetLights();

   private: 
      unsigned long _LightsOnSchedule[4];
      unsigned long _LightsOffSchedule[4];
      uint8_t _LIGHT_1_PIN;
      uint8_t _LIGHT_2_PIN;
      uint8_t _LIGHT_3_PIN;
      uint8_t _LIGHT_4_PIN;

      bool _StartSequenceStarted = false;


      int _msBetweenLights = 1000;
};

extern LightsControllerClass LightsController;

#endif