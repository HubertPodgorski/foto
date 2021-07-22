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
///   Main entry-point for this application. This function should be called once every main loop.
///   It will check if any new interrupts were saved from the sensors, and handle them if this
///   the case. All timing related data and also fault handling of the dogs is done in this
///   function.
/// </summary>
void RaceHandlerClass::Main() {
   //Don't handle anything if race is stopped
   if (RaceState == STOP) {
      return;
   }

   //If queue is not empty, we have work to do
   if (!_QueueEmpty()) {
      //Get next record from queue
      SensorTriggerRecord SensorTriggerRecord = _QueuePop();
      //If the transition string is not empty and is was not updated for 2 seconds then we have to clear it.
      if (_Transition.length() != 0 && (micros() - LastTransitionStringUpdate) > 2000000) {
         _Transition = "";
      } if (_Transition.length() == 0) {
         _AreGatesClear = true;
      }

      // FIX LOGGING LATER
      // ESP_LOGD(__FILE__, "S%i|T:%li|St:%i", SensorTriggerRecord.SensorNumber, SensorTriggerRecord.TriggerTime - _RaceStartTime, SensorTriggerRecord.SensorState);
      // ESP_LOGD(__FILE__, "bGatesClear: %i", _AreGatesClear);


      // <<<<<<<<<<<<<<<<<< DONE HERE >>>>>>>>>>>>>>>>


      // RERUNS TO BE DONE


      NextDogIndex = CurrentDogIndex + 1;

      //Handle sensor 1 events (handlers side)
      if (SensorTriggerRecord.SensorNumber == 1 && _AreGatesClear && SensorTriggerRecord.SensorState == 1) { //Only if gates are clear nd act on HIGH events (beam broken)
         //First check if we don't have a fault situation
         //Special fault handling for dog 0 when it's not yet in the lane
         if (CurrentDogIndex == 0 && _DogRunDirection == GOINGIN && SensorTriggerRecord.TriggerTime < _PerfectCrossingTime) {
            //Dog 0 is too early!
            SetDogFault(CurrentDogIndex, ON);
            // TODO: handle logging
            // ESP_LOGD(__FILE__, "F! D:%i!", CurrentDogIndex);
            _CrossingTimes[CurrentDogIndex][_DogRunCounters[CurrentDogIndex]] = SensorTriggerRecord.TriggerTime - _PerfectCrossingTime;
            _DogEnterTimes[CurrentDogIndex] = SensorTriggerRecord.TriggerTime;

            //Check if this is a next dog which is too early (we are expecting a dog to come back)
         } else if (_DogRunDirection == COMINGBACK) {
            //This dog is too early!
            //We don't increase the dog number at this point. The transition string further down in the code will determine whether current dog came in or not.
            //Set fault light for next dog.
            SetDogFault(NextDogIndex, ON);

            //For now we assume dogs crossed more or less at the same time.
            //It is very unlikely that a next dog clears the sensors before the previous dog crosses them (this would be a veeery early crossing).
            _DogExitTimes[CurrentDogIndex] = SensorTriggerRecord.TriggerTime;
            _DogTimes[CurrentDogIndex][_DogRunCounters[CurrentDogIndex]] = SensorTriggerRecord.TriggerTime - _DogEnterTimes[CurrentDogIndex];

            //Handle next dog
            _DogEnterTimes[NextDogIndex] = SensorTriggerRecord.TriggerTime;
            
            // TODO: handle logging
            // ESP_LOGD(__FILE__, "F! D:%i!", NextDogIndex);
         }

         //Normal race handling (no faults)
         if (_DogRunDirection == GOINGIN) {
            //Store crossing time
            _CrossingTimes[CurrentDogIndex][_DogRunCounters[CurrentDogIndex]] = SensorTriggerRecord.TriggerTime - _PerfectCrossingTime;

            //If this dog is doing a rerun we have to turn the error light for this dog off
            if (_RerunBusy) {
               //remove fault for this dog
               SetDogFault(CurrentDogIndex, OFF);
            }
         }
      }

      //Handle sensor 2 (box side)
      // Only if gates are clear And only if sensor is HIGH
      if (SensorTriggerRecord.SensorNumber == 2 && _AreGatesClear && SensorTriggerRecord.SensorState == 1) { 
         if (_DogRunDirection != COMINGBACK) {
            /* Gates were clear, we weren't expecting a dog back, but one came back.
            This means we missed the dog going in,
            most likely due to perfect crossing where next dog was faster than previous dog,
            and thus passed through sensors unseen */
            //Set enter time for this dog to exit time of previous dog
            _DogEnterTimes[CurrentDogIndex] = _DogExitTimes[PreviousDog];
            
            // TODO: do logging
            // ESP_LOGD(__FILE__, "Invisible dog came back!");
         }

         //Check if current dog has a fault
         //TODO: The current dog could also have a fault which is not caused by being too early (manually triggered fault).
         //We should store the fault type also so we can check if the dog was too early or not.

         //If dog is not 1st dog and current dog has fault and S2 is trigger less than 2s after current dog's enter time
         //Then we know It's actually the previous dog who's still coming back (current dog was way too early).
         if (CurrentDogIndex != 0 && _DogFaults[CurrentDogIndex] && (SensorTriggerRecord.TriggerTime - _DogEnterTimes[CurrentDogIndex]) < 2000000) {
            //Current dog had a fault (was too early), so we need to modify the previous dog crossing time (we didn't know this before)
            //Update exit and total time of previous dog
            _DogExitTimes[PreviousDog] = SensorTriggerRecord.TriggerTime;
            _DogTimes[PreviousDog][_DogRunCounters[PreviousDog]] = _DogExitTimes[PreviousDog] - _DogEnterTimes[PreviousDog];

            //And update crossing time of this dog (who is in fault)
            _CrossingTimes[CurrentDogIndex][_DogRunCounters[CurrentDogIndex]] = _DogEnterTimes[CurrentDogIndex] - _DogExitTimes[PreviousDog];

            //Filter out S2 HIGH signals that are < 2 seconds after dog enter time
         } else if ((SensorTriggerRecord.TriggerTime - _DogEnterTimes[CurrentDogIndex]) > 2000000) {
            //Normal handling for dog coming back
            _DogExitTimes[CurrentDogIndex] = SensorTriggerRecord.TriggerTime;
            _DogTimes[CurrentDogIndex][_DogRunCounters[CurrentDogIndex]] = SensorTriggerRecord.TriggerTime - _DogEnterTimes[CurrentDogIndex];
            //The time the dog came OUT is also the perfect crossing time
            _PerfectCrossingTime = SensorTriggerRecord.TriggerTime;


            //If this is the 4th dog and there is no fault we have to stop the race
            // OR if the rerun sequence was started but no faults exist anymore
            if ((CurrentDogIndex == 3 && _Fault == false && _RerunBusy == false) || (_RerunBusy == true && _Fault == false)) {
               StopRace(SensorTriggerRecord.TriggerTime);
               
               // TODO: handle logging
               ESP_LOGD(__FILE__, "Last Dog: %i|ENT:%lu|EXIT:%lu|TOT:%lu", CurrentDogIndex, _DogEnterTimes[CurrentDogIndex], _DogExitTimes[CurrentDogIndex], _DogTimes[CurrentDogIndex][_DogRunCounters[CurrentDogIndex]]);
            }
            else if ((CurrentDogIndex == 3 && _Fault == true && _RerunBusy == false) //If current dog is dog 4 and a fault exists, we have to initiate rerun sequence
                     || _RerunBusy == true)                                       //Or if rerun is busy (and faults still exist)
            {
               //Dog 3 came in but there is a fault, we have to initiate the rerun sequence
               _RerunBusy = true;
               //Reset timers for this dog
               _DogEnterTimes[NextDogIndex] = SensorTriggerRecord.TriggerTime;
               _DogExitTimes[NextDogIndex] = 0;
               //Increase run counter for this dog
               _DogRunCounters[NextDogIndex]++;
               ESP_LOGI(__FILE__, "RR%i", NextDogIndex);
            }
            else
            {
               //Store next dog enter time
               _DogEnterTimes[NextDogIndex] = SensorTriggerRecord.TriggerTime;
            }
         }
      }

      /***********************************
       * The code below handles what we call the 'transition string'
       * It is an algorithm which saves all sensor events in sequence, until it recognizes a pattern.
       * We have 2 sensor columns: the handler side column, and the box side column
       * To indicate the handler side column, we use the letter A
       * To indicate the box side column, we use the letter B
       * A HIGH sensor reading (beam broken), will be represented by an upper case character
       * A LOW sensor reading (beam not broken), will be represented by a lower case character
       * e.g.: A --> handler side HIGH reading, b --> box side LOW reading, etc...
       * 
       * We chain these characters up to get our 'transition string'
       * Then we check this string to determine what happened
       * For example:
       * 'ABab'
       *    --> Handler side HIGH, box side HIGH, handler side LOW, box side LOW
       *    --> This tells us ONE dog passed the gates in the direction of the box
       * 'BAba'
       *    --> Box side HIGH, handler side HIGH, box side LOW, handler side LOW
       *    --> This tells us ONE dog passed the gates in the direction of the handler
       * 'BAab'
       *    --> Box side HIGH, handler side HIGH, handler side LOW, box side LOW
       *    --> This tells us TWO dogs crossed the gates simultaneously, and the dog going to the box was the last to leave the gates
      ***********************************/

      //Add trigger record to transition string
      _AddToTransitionString(SensorTriggerRecord);

      //Check if the transition string up till now tells us the gates are clear
      String strLast2TransitionChars = _Transition.substring(_Transition.length() - 2);
      if (_Transition.length() == 0                                             //String might still be empty, in which case the gates were clear
          || (strLast2TransitionChars == "ab" || strLast2TransitionChars == "ba")) //Sensors going low in either direction indicate gates are clear
      {
         //The gates are clear, set boolean
         _AreGatesClear = true;

         //Print the transition string up til now for debugging purposes
         ESP_LOGD(__FILE__, "Tstring: %s", _Transition.c_str());

         //Only check transition string when gates are clear
         //TODO: If transistion string is 3 or longer but actually more events are coming related to same transition, these are not considered.
         if (_Transition.length() > 3) //And if transistion string is at least 4 characters long
         {
            //Transition string is 4 characters or longer
            //So we can check what happened
            if (_Transition == "ABab") //Dog going to box
            {
               //Change dog state to coming back
               _ChangeDogState(COMINGBACK);
            }
            else if (_Transition == "BAba") //Dog coming back
            {
               //Normal handling, change dog state to GOING IN
               _ChangeDogState(GOINGIN);
               //Set next dog active
               _ChangeDogNumber(NextDogIndex);
            }
            else if (_Transition == "BbAa")
            {
               //Transistion string BbAa indicates small object has passed through sensors
               //Most likely dog spat ball
               ESP_LOGI(__FILE__, "Spat ball detected?!");
               SetDogFault(CurrentDogIndex, ON);
            }
            else //Transition string indicated something other than dog coming back or dog going in, it means 2 dogs must have passed
            {
               //Transition string indicates more than 1 dog passed
               //We increase the dog number
               _ChangeDogNumber(NextDogIndex);

               //First check if no error was set for next dog (too early)
               if (_DogFaults[CurrentDogIndex])
               {
                  //This dog was too early, but since have a simultaneous crossing we don't know the crossing time.
                  //Set it to 0 for now
               }

               // and set perfect crossing time for new dog
               _ChangeDogState(COMINGBACK);
               _CrossingTimes[CurrentDogIndex][_DogRunCounters[CurrentDogIndex]] = 0;
               _DogEnterTimes[CurrentDogIndex] = _DogExitTimes[PreviousDog];
            }
         }
         _Transition = "";
      }
      else
      {
         _AreGatesClear = false;
      }
   }

   //Update racetime
   if (RaceState == RUNNING)
   {
      if (GET_MICROS > _RaceStartTime)
      {
         _RaceTime = GET_MICROS - _RaceStartTime;
      }
   }

   //Check for faults, loop through array of dogs checking for faults
   _Fault = false;
   for (auto Fault : _DogFaults)
   {
      if (Fault)
      {
         //At least one dog with fault is found, set general fault value to true
         _Fault = true;
         break;
      }
   }
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
///   Sets dog fault for given dog number to given state.
/// </summary>
///
/// <param name="DogIndex"> Zero-based index of the dog number. </param>
/// <param name="State">      The state. </param>
void RaceHandlerClass::SetDogFault(uint8_t DogIndex, DogFaults State) {
   //Don't process any faults when race is not running
   if (RaceState == STOP) {
      return;
   }

   bool Fault;
   //Check if we have to toggle
   if (State == TOGGLE) {
      Fault = !_DogFaults[DogIndex];
   } else {
      Fault = State;
   }

   //Set fault to specified value for relevant dog
   _DogFaults[DogIndex] = Fault;

   
   // <<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>
   // TODO: SET LIGHT FAULTS
   //If fault is true, set light to on and set general value fault variable to true
   // if (Fault) {
   //    LightsController.ToggleFaultLight(DogIndex, LightsController.ON);
   //    _Fault = true;
   //    ESP_LOGI(__FILE__, "D%iF1", DogIndex);
   // } else {
   //    //If fault is false, turn off fault light for this dog
   //    LightsController.ToggleFaultLight(DogIndex, LightsController.OFF);
   //    ESP_LOGI(__FILE__, "D%iF0", DogIndex);
   // }
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

/// <summary>
///   Starts the timers. Should be called once GREEN light comes ON.
/// </summary>
void RaceHandlerClass::StartTimers() {
   _ChangeRaceState(RACING);
}

/// <summary>
///   Stops a race.
/// </summary>
void RaceHandlerClass::StopRace() {
   this->StopRace(micros());
}

/// <summary>
///   Stops a race.
/// </summary>
/// <param name="StopTime">   The time in microseconds at which the race stopped. </param>
void RaceHandlerClass::StopRace(unsigned long StopTime) {
   if (RaceState == RUNNING) {
      //Race is running, so we have to record the EndTime
      _RaceEndTime = StopTime;
      _RaceTime = _RaceEndTime - _RaceStartTime;
   }
   _ChangeRaceState(STOP);

   _HistoricRaceData[_CurrentRaceId] = GetRaceData(_CurrentRaceId);
}

/// <summary>
///   Gets race data for the current race
/// </summary>
///
/// <returns>
///   The race data struct
/// </returns>
RaceData RaceHandlerClass::GetRaceData() {
   return GetRaceData(_CurrentRaceId);
}

/// <summary>
///   Gets race data for given race ID
/// </summary>

/// <param name="RaceId">The ID for the race you want the data for</param>
///
/// <returns>
///  Race data struct
/// </returns>
RaceData RaceHandlerClass::GetRaceData(unsigned int RaceId) {
   RaceData RequestedRaceData;

   if (RaceId == _CurrentRaceId) {
      //We need to return data for the current dace
      RequestedRaceData.Id = _CurrentRaceId;
      RequestedRaceData.StartTime = _RaceStartTime / 1000;
      RequestedRaceData.EndTime = _RaceEndTime / 1000;
      RequestedRaceData.ElapsedTime = _RaceTime / 1000;
      // TODO: do logging
      //Serial.printf("Elapsed1: %lu - %lu = %lu\r\n", micros(), _lRaceStartTime, _lRaceTime);
      //Serial.printf("Elapsed2: %lu - %lu = %lu\r\n", GET_MICROS, _lRaceStartTime, _lRaceTime);
      RequestedRaceData.TotalCrossingTime = this->GetTotaCrossingTimeMillis();
      RequestedRaceData.RaceState = RaceState;

      //Get Dog info
      for (uint8_t dogIndex = 0; dogIndex < 4; dogIndex++) {
         RequestedRaceData.DogData[dogIndex].DogNumber = dogIndex;

         // WHAT IS i2?
         for (uint8_t i2 = 0; i2 < 4; i2++) {
            RequestedRaceData.DogData[dogIndex].Timing[i2].Time = GetDogTimeMillis(dogIndex, i2);
            RequestedRaceData.DogData[dogIndex].Timing[i2].CrossingTime = GetCrossingTimeMillis(dogIndex, i2);
         }

         RequestedRaceData.DogData[dogIndex].Fault = _DogFaults[dogIndex];
         RequestedRaceData.DogData[dogIndex].Running = (CurrentDog == dogIndex);
      }
   } else {
      RequestedRaceData = _HistoricRaceData[RaceId];
   }

   return RequestedRaceData;
}

/// <summary>
///   Gets dogs crossing time. Keep in mind each dog can have multiple runs (reruns for faultS).
/// </summary>
///
/// <param name="DogIndex"> Zero-based index of the dog number. </param>
/// <param name="RunNumber"> Zero-based index of the run number. If -1 is passed, this function
///                           will alternate between each run we have for the dog, passing a new
///                           run every 2 seconds. If -2 is passed, the last run number we have
///                           for this dog will be passed. </param>
///
/// <returns>
///   The dog time in seconds with 2 decimals.
/// </returns>
String RaceHandlerClass::GetCrossingTime(uint8_t DogIndex, int8_t RunNumber) {
   double CrossingTime = 0;
   char CharCrossingTime[8];
   String StringCrossingTime;
   long CrossingTimeMillis = GetCrossingTimeMillis(DogIndex, RunNumber);
   CrossingTime = CrossingTimeMillis / 1000.0;

   if (CrossingTime < 0) {
      CrossingTime = fabs(CrossingTime);
      StringCrossingTime = "-";
   } else {
      StringCrossingTime = "+";
   }

   dtostrf(CrossingTime, 7, 3, CharCrossingTime);
   StringCrossingTime += CharCrossingTime;

   return StringCrossingTime;
}

unsigned long RaceHandlerClass::GetCrossingTimeMillis(uint8_t DogIndex, int8_t RunNumber) {
   long CrossingTime = 0;
   if (_DogRunCounters[DogIndex] > 0) {
      //We have multiple times for this dog.
      //if run number is -1 (unspecified), we have to cycle throug them
      if (RunNumber == -1) {
         auto &LastReturnedTimeStamp = _LastDogTimeReturnTimeStamp[DogIndex];
         RunNumber = _LastReturnedRunNumber[DogIndex];
         if ((millis() - LastReturnedTimeStamp) > 2000) {
            if (RunNumber == _DogRunCounters[DogIndex]) {
               RunNumber = 0;
            } else {
               RunNumber++;
            }

            LastReturnedTimeStamp = millis();
         }

         _LastReturnedRunNumber[DogIndex] = RunNumber;
      } else if (RunNumber == -2) {
         //if RunNumber is -2 it means we should return the last one
         RunNumber = _DogRunCounters[DogIndex];
      }
   } else if (RunNumber < 0) {
      RunNumber = 0;
   }

   CrossingTime = _CrossingTimes[DogIndex][RunNumber] / 1000;

   return CrossingTime;
}

/// <summary>
///   Gets dog time, time since dog entered if dog is still running, final time if dog is already
///   back in. Keep in mind each dog can have multiple runs (reruns for faultS).
/// </summary>
///
/// <param name="DogIndex"> Zero-based index of the dog number. </param>
/// <param name="RunNumber"> Zero-based index of the run number. If -1 is passed, this function
///                           will alternate between each run we have for the dog, passing a new
///                           run every 2 seconds. If -2 is passed, the last run number we have
///                           for this dog will be passed. </param>
///
/// <returns>
///   The dog time in seconds with 2 decimals.
/// </returns>
double RaceHandlerClass::GetDogTime(uint8_t DogIndex, int8_t RunNumber) {
   double DogTime;
   unsigned long DogTimeMillis = GetDogTimeMillis(DogIndex, RunNumber);
   DogTime = DogTimeMillis / 1000.0;

   return DogTime;
}

unsigned long RaceHandlerClass::GetDogTimeMillis(uint8_t DogIndex, int8_t RunNumber) {
   unsigned long DogTimeMillis = 0;

   if (_DogRunCounters[DogIndex] > 0) {
      //We have multiple times for this dog.
      //if run number is -1 (unspecified), we have to cycle throug them
      if (RunNumber == -1) {
         auto &LastReturnedTimeStamp = _LastDogTimeReturnTimeStamp[DogIndex];

         RunNumber = LastReturnedRunNumber[DogIndex];

         if ((millis() - LastReturnedTimeStamp) > 2000) {
            if (RunNumber == _DogRunCounters[DogIndex]) {
               RunNumber = 0;
            } else {
               RunNumber++;
            }

            LastReturnedTimeStamp = millis();
         }

         LastReturnedRunNumber[DogIndex] = RunNumber;
      } else if (RunNumber == -2) {
         //if RunNumber is -2 it means we should return the last one
         RunNumber = _DogRunCounters[DogIndex];
      }
   } else if (RunNumber < 0) {
      RunNumber = 0;
   }

   //First check if we have final time for the requested dog number
   if (_DogTimes[DogIndex][RunNumber] > 0) {
      DogTimeMillis = _DogTimes[DogIndex][RunNumber] / 1000;


   // Then check if the requested dog is perhaps running (and coming back) so we can return the time so far
   // And if requested run number is lower then number of times dog has run 
   } else if ((RaceState == RACING && CurrentDog == DogIndex && _DogRunDirection == COMINGBACK) && RunNumber <= _DogRunCounters[DogIndex]){
      DogTimeMillis = (micros() - _DogEnterTimes[DogIndex]) / 1000;
   }

   //Fixes issue 7 (https://github.com/vyruz1986/FlyballETS-Software/issues/7)
   //Only deduct crossing time if it is positive
   if (_CrossingTimes[DogIndex][RunNumber] > 0 && DogTimeMillis > (_CrossingTimes[DogIndex][RunNumber] / 1000)) {
      DogTimeMillis -= (_CrossingTimes[DogIndex][RunNumber] / 1000);
   }

   return DogTimeMillis;
}

/// <summary>
///   Gets total crossing time. This will return the total crossing time of all dogs (and reruns
///   if applicable). It allows the user to easily calculate the theoretical best time of the
///   team by subtracting this number from the total team time.
/// </summary>
///
/// <returns>
///   The total crossing time in milliseconds
/// </returns>
long RaceHandlerClass::GetTotaCrossingTimeMillis() {
   long TotalCrossingTime = 0;

   for (auto &Dog : _CrossingTimes) {
      for (auto &Time : Dog) {
         TotalCrossingTime += Time;
      }
   }
   return TotalCrossingTime / 1000;
}

/// <summary>
///   Changes race state, if byNewRaceState is different from current one.
/// </summary>
///
/// <param name="byNewRaceState">   New race state. </param>
void RaceHandlerClass::_ChangeRaceState(RaceStates NewRaceState) {
   //First check if the new state (this function could be called superfluously)
   if (RaceState != NewRaceState) {
      PreviousRaceState = RaceState;
      RaceState = NewRaceState;
   }
}

/// <summary>
///   Determines if the interrupt buffer queue is empty.
/// </summary>
///
/// <returns>
///   true if it is empty, false if it is not.
/// </returns>
bool RaceHandlerClass::_QueueEmpty() {
   //This function checks if queue is empty.
   //This is determined by comparing the read and write index.
   //If they are equal, it means we have cought up reading and the queue is 'empty' (the array is not really emmpty...)
   if (_QueueReadIndex == _QueueWriteIndex) {
      return true;
   } else {
      return false;
   }
}

RaceHandlerClass::SensorTriggerRecord RaceHandlerClass::_QueuePop() {
   //This function returns the next record of the interrupt queue
   SensorTriggerRecord NextRecord = _SensorTriggerQueue[_QueueReadIndex];

   //Read index has to be increased, check it we should wrap-around
   if (_QueueReadIndex == TRIGGER_QUEUE_LENGTH - 1) //(sizeof(_STriggerQueue) / sizeof(*_STriggerQueue) - 1)) {
      //Write index has reached end of array, start at 0 again
      _iQueueReadIndex = 0;
   } else {
      //End of array not yet reached, increase index by 1
      _QueueReadIndex++;
   }

   return NextRecord;
}

RaceHandlerClass RaceHandler;