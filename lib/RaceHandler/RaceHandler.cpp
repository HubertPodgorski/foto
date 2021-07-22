#include "RaceHandler.h"

/// <summary>
///   Initialises this object andsets all counters to 0.
/// </summary>
///
/// <param name="iS1Pin">  Zero-based index of the S1 pin. </param>
/// <param name="iS2Pin">  Zero-based index of the S2 pin. </param>
void RaceHandlerClass::init(uint8_t Sensor1Pin, uint8_t Sensor2Pin)
{
   //Start in STOP race state
   _ChangeRaceState(STOP);
   _Sensor1Pin = Sensor1Pin;
   _Sensor2Pin = Sensor2Pin;

   ResetRace();
}

/// <summary>
///   ISR function for sensor 1, this function will record the sensor number, microseconds and
///   state (HIGH/LOW) of the sensor in the interrupt queue.
/// </summary>
void RaceHandlerClass::TriggerSensor1()
{
   if (RaceState == STOP)
   {
      return;
   }
   uint8_t sensorNumber = 1;
   _QueuePush({sensorNumber, micros(), digitalRead(_Sensor1Pin)});
}

/// <summary>
///   ISR function for sensor 2, this function will record the sensor number, microseconds and
///   state (HIGH/LOW) of the sensor in the interrupt queue.
/// </summary>
void RaceHandlerClass::TriggerSensor2()
{
   if (RaceState == STOP)
   {
      return;
   }
   uint8_t sensorNumber = 2;
   _QueuePush({sensorNumber, micros(), digitalRead(_Sensor2Pin)});
}

/// <summary>
///   Pushes an interrupt trigger record to the back of the interrupt buffer.
/// </summary>
///
/// <param name="_InterruptTrigger">   The interrupt trigger record. </param>
void RaceHandlerClass::_QueuePush(RaceHandlerClass::SensorTriggerRecord _InterruptTrigger)
{
   //Add record to queue
   _SensorTriggerQueue[_QueueWriteIndex] = _InterruptTrigger;

   //Write index has to be increased, check it we should wrap-around
   if (_QueueWriteIndex == TRIGGER_QUEUE_LENGTH - 1) //(sizeof(_STriggerQueue) / sizeof(*_STriggerQueue) - 1))
   {
      //Write index has reached end of array, start at 0 again
      _QueueWriteIndex = 0;
   }
   else
   {
      //End of array not yet reached, increase index by 1
      _QueueWriteIndex++;
   }
}

/// <summary>
///   Resets the race, this function should be called to reset all timers to 0 and prepare the
///   software for starting a next race.
/// </summary>
void RaceHandlerClass::ResetRace() {
   RaceState = STOP;
   _QueueReadIndex = 0;
   _QueueWriteIndex = 0;
}

RaceHandlerClass RaceHandler;