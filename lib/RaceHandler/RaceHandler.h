#include "Arduino.h"

class RaceHandlerClass
{
   public:
      void init(uint8_t Sensor1Pin, uint8_t Sensor2Pin);

      enum RaceStates {
         RACING,
         STOP
      };

      RaceStates RaceState = STOP;

      void TriggerSensor1();
      void TriggerSensor2();
      void ResetRace();


   private:
      uint8_t _Sensor1Pin;
      uint8_t _Sensor2Pin;

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
};

extern RaceHandlerClass RaceHandler;