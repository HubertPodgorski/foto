#ifndef _RACEHANDLER_h
#define _RACEHANDLER_h

#include "Arduino.h"
#include "Structs.h"

#define NUM_HISTORIC_RACE_RECORDS 100

class RaceHandlerClass {
   public:
   
      uint8_t CurrentDogIndex;
      uint8_t PreviousDogIndex;
      uint8_t NextDogIndex;
      void init(uint8_t Sensor1Pin, uint8_t Sensor2Pin);

      enum RaceStates {
         STARTING,
         RACING,
         STOP
      };

      enum DogFaults
      {
         OFF,
         ON,
         TOGGLE
      };

      RaceStates RaceState = STOP;
      RaceStates PreviousRaceState = STOP;

      void TriggerSensor1();
      void TriggerSensor2();
      void ResetRace();
      void StartTimers();
      void Main();
      void SetDogFault(uint8_t DogIndex, DogFaults State = TOGGLE);
      void StopRace();
      void StopRace(unsigned long StopTime);
      double GetRaceTime();
      RaceData GetRaceData();
      RaceData GetRaceData(unsigned int RaceId);
      long GetTotalCrossingTimeMillis();
      double GetTotalCrossingTime();
      double GetDogTime(uint8_t DogIndex, int8_t RunNumber = -1);
      unsigned long GetDogTimeMillis(uint8_t DogIndex, int8_t RunNumber = -1);
      String GetCrossingTime(uint8_t DogIndex, int8_t RunNumber = -1);
      unsigned long GetCrossingTimeMillis(uint8_t DogIndex, int8_t RunNumber = -1);
      void StartRace();
      String GetRerunInfo(uint8_t DogIndex);

      String GetRaceStateString();


   private:
      bool _Fault;
      uint8_t _Sensor1Pin;
      uint8_t _Sensor2Pin;
      unsigned long _LastTransitionStringUpdate;
      unsigned long _PerfectCrossingTime;
      bool _AreGatesClear = false;
      bool _DogFaults[4];
      long _CrossingTimes[4][4];
      uint8_t _DogRunCounters[4]; //Number of (re-)runs for each dog
      unsigned long _DogEnterTimes[4];
      unsigned long _DogExitTimes[4];
      unsigned long _DogTimes[4][4];
      bool _RerunBusy;
      unsigned long long _RaceEndTime;
      unsigned long long _RaceTime;
      unsigned long long _RaceStartTime;
      unsigned int _CurrentRaceId;
      unsigned long _LastDogTimeReturnTimeStamp[4];
      uint8_t _LastReturnedRunNumber[4];

      String _Transition;

      enum _DogRunDirections
      {
         GOINGIN,
         COMINGBACK
      };
      _DogRunDirections _DogRunDirection;

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
      void _ChangeDogIndex(uint8_t _NewDogIndex);

      RaceData _HistoricRaceData[NUM_HISTORIC_RACE_RECORDS];

      void _AddToTransitionString(SensorTriggerRecord _InterruptTrigger);

      void _ChangeDogRunDirection(_DogRunDirections NewDogRunDirection);
};

extern RaceHandlerClass RaceHandler;

#endif