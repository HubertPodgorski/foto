#ifndef _RACEHANDLER_h
#define _RACEHANDLER_h

#include "Arduino.h"

class RaceHandlerClass
{
   public:
      void init(uint8_t Sensor1Pin, uint8_t Sensor2Pin);

      enum RaceStates {
         STARTING,
         RACING,
         STOP
      };

      RaceStates RaceState = STOP;
      RaceStates PreviousRaceState = STOP;

      void TriggerSensor1();
      void TriggerSensor2();
      void ResetRace();
      void StartTimers();
      void Main();


   private:
      uint8_t _Sensor1Pin;
      uint8_t _Sensor2Pin;
      unsigned long _LastTransitionStringUpdate;
      
      bool _AreGatesClear = false;

      String _Transition;
      
      struct SensorTriggerRecord {
         volatile uint8_t sensorNumber;
         volatile long long triggerTime;
         volatile int sensorState;
      };

      #define TRIGGER_QUEUE_LENGTH 50
      SensorTriggerRecord _SensorTriggerQueue[TRIGGER_QUEUE_LENGTH];

      volatile uint8_t _QueueReadIndex;
      volatile uint8_t _QueueWriteIndex;

      void _QueuePush(SensorTriggerRecord _InterruptTrigger);
      void _ChangeRaceState(RaceStates _NewRaceState);
      bool _QueueEmpty();
      SensorTriggerRecord _QueuePop();
};

extern RaceHandlerClass RaceHandler;

#endif