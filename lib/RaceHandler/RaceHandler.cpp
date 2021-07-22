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
      // ESP_LOGD(__FILE__, "S%i|T:%li|St:%i", SensorTriggerRecord.SensorNumber, SensorTriggerRecord.TriggerTime - _lRaceStartTime, SensorTriggerRecord.SensorState);
      // ESP_LOGD(__FILE__, "bGatesClear: %i", _AreGatesClear);


      // <<<<<<<<<<<<<<<<<< DONE HERE >>>>>>>>>>>>>>>>


      // RERUNS TO BE DONE
      NextDogIndex = CurrentDogIndex + 1;

      //Handle sensor 1 events (handlers side)
      if (SensorTriggerRecord.SensorNumber == 1 && _AreGatesClear //Only if gates are clear
          && SensorTriggerRecord.SensorState == 1)              //And act on HIGH events (beam broken)
      {
         //First check if we don't have a fault situation
         //Special fault handling for dog 0 when it's not yet in the lane
         if (CurrentDogIndex == 0 && _byDogState == GOINGIN && SensorTriggerRecord.TriggerTime < _lPerfectCrossingTime)
         {
            //Dog 0 is too early!
            SetDogFault(CurrentDogIndex, ON);
            ESP_LOGD(__FILE__, "F! D:%i!", CurrentDogIndex);
            _lCrossingTimes[CurrentDogIndex][_iDogRunCounters[CurrentDogIndex]] = SensorTriggerRecord.TriggerTime - _lPerfectCrossingTime;
            _lDogEnterTimes[CurrentDogIndex] = SensorTriggerRecord.TriggerTime;
         }
         //Check if this is a next dog which is too early (we are expecting a dog to come back)
         else if (_byDogState == COMINGBACK)
         {
            //This dog is too early!
            //We don't increase the dog number at this point. The transition string further down in the code will determine whether current dog came in or not.
            //Set fault light for next dog.
            SetDogFault(NextDogIndex, ON);

            //For now we assume dogs crossed more or less at the same time.
            //It is very unlikely that a next dog clears the sensors before the previous dog crosses them (this would be a veeery early crossing).
            _lDogExitTimes[CurrentDogIndex] = SensorTriggerRecord.TriggerTime;
            _lDogTimes[CurrentDogIndex][_iDogRunCounters[CurrentDogIndex]] = SensorTriggerRecord.TriggerTime - _lDogEnterTimes[CurrentDogIndex];

            //Handle next dog
            _lDogEnterTimes[NextDogIndex] = SensorTriggerRecord.TriggerTime;
            ESP_LOGD(__FILE__, "F! D:%i!", NextDogIndex);
         }

         //Normal race handling (no faults)
         if (_byDogState == GOINGIN)
         {
            //Store crossing time
            _lCrossingTimes[CurrentDogIndex][_iDogRunCounters[CurrentDogIndex]] = SensorTriggerRecord.TriggerTime - _lPerfectCrossingTime;

            //If this dog is doing a rerun we have to turn the error light for this dog off
            if (_bRerunBusy)
            {
               //remove fault for this dog
               SetDogFault(CurrentDogIndex, OFF);
            }
         }
      }

      //Handle sensor 2 (box side)
      if (SensorTriggerRecord.SensorNumber == 2 && _AreGatesClear //Only if gates are clear
          && SensorTriggerRecord.SensorState == 1)              //And only if sensor is HIGH
      {
         if (_byDogState != COMINGBACK)
         {
            /* Gates were clear, we weren't expecting a dog back, but one came back.
            This means we missed the dog going in,
            most likely due to perfect crossing were next dog was faster than previous dog,
            and thus passed through sensors unseen */
            //Set enter time for this dog to exit time of previous dog
            _lDogEnterTimes[CurrentDogIndex] = _lDogExitTimes[iPreviousDog];
            ESP_LOGD(__FILE__, "Invisible dog came back!");
         }

         //Check if current dog has a fault
         //TODO: The current dog could also have a fault which is not caused by being too early (manually triggered fault).
         //We should store the fault type also so we can check if the dog was too early or not.
         if (CurrentDogIndex != 0                                                           //If dog is not 1st dog
             && _bDogFaults[CurrentDogIndex]                                                //and current dog has fault
             && (SensorTriggerRecord.TriggerTime - _lDogEnterTimes[CurrentDogIndex]) < 2000000) //And S2 is trigger less than 2s after current dog's enter time
                                                                                        //Then we know It's actually the previous dog who's still coming back (current dog was way too early).
         {
            //Current dog had a fault (was too early), so we need to modify the previous dog crossing time (we didn't know this before)
            //Update exit and total time of previous dog
            _lDogExitTimes[iPreviousDog] = SensorTriggerRecord.TriggerTime;
            _lDogTimes[iPreviousDog][_iDogRunCounters[iPreviousDog]] = _lDogExitTimes[iPreviousDog] - _lDogEnterTimes[iPreviousDog];

            //And update crossing time of this dog (who is in fault)
            _lCrossingTimes[CurrentDogIndex][_iDogRunCounters[CurrentDogIndex]] = _lDogEnterTimes[CurrentDogIndex] - _lDogExitTimes[iPreviousDog];
         }
         else if ((SensorTriggerRecord.TriggerTime - _lDogEnterTimes[CurrentDogIndex]) > 2000000) //Filter out S2 HIGH signals that are < 2 seconds after dog enter time
         {
            //Normal handling for dog coming back
            _lDogExitTimes[CurrentDogIndex] = SensorTriggerRecord.TriggerTime;
            _lDogTimes[CurrentDogIndex][_iDogRunCounters[CurrentDogIndex]] = SensorTriggerRecord.TriggerTime - _lDogEnterTimes[CurrentDogIndex];
            //The time the dog came in is also the perfect crossing time
            _lPerfectCrossingTime = SensorTriggerRecord.TriggerTime;

            if ((CurrentDogIndex == 3 && _bFault == false && _bRerunBusy == false) //If this is the 4th dog and there is no fault we have to stop the race
                || (_bRerunBusy == true && _bFault == false))                  //Or if the rerun sequence was started but no faults exist anymore
            {
               StopRace(SensorTriggerRecord.TriggerTime);
               ESP_LOGD(__FILE__, "Last Dog: %i|ENT:%lu|EXIT:%lu|TOT:%lu", CurrentDogIndex, _lDogEnterTimes[CurrentDogIndex], _lDogExitTimes[CurrentDogIndex], _lDogTimes[CurrentDogIndex][_iDogRunCounters[CurrentDogIndex]]);
            }
            else if ((CurrentDogIndex == 3 && _bFault == true && _bRerunBusy == false) //If current dog is dog 4 and a fault exists, we have to initiate rerun sequence
                     || _bRerunBusy == true)                                       //Or if rerun is busy (and faults still exist)
            {
               //Dog 3 came in but there is a fault, we have to initiate the rerun sequence
               _bRerunBusy = true;
               //Reset timers for this dog
               _lDogEnterTimes[NextDogIndex] = SensorTriggerRecord.TriggerTime;
               _lDogExitTimes[NextDogIndex] = 0;
               //Increase run counter for this dog
               _iDogRunCounters[NextDogIndex]++;
               ESP_LOGI(__FILE__, "RR%i", NextDogIndex);
            }
            else
            {
               //Store next dog enter time
               _lDogEnterTimes[NextDogIndex] = SensorTriggerRecord.TriggerTime;
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
               if (_bDogFaults[CurrentDogIndex])
               {
                  //This dog was too early, but since have a simultaneous crossing we don't know the crossing time.
                  //Set it to 0 for now
               }

               // and set perfect crossing time for new dog
               _ChangeDogState(COMINGBACK);
               _lCrossingTimes[CurrentDogIndex][_iDogRunCounters[CurrentDogIndex]] = 0;
               _lDogEnterTimes[CurrentDogIndex] = _lDogExitTimes[iPreviousDog];
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
      if (GET_MICROS > _lRaceStartTime)
      {
         _lRaceTime = GET_MICROS - _lRaceStartTime;
      }
   }

   //Check for faults, loop through array of dogs checking for faults
   _bFault = false;
   for (auto bFault : _bDogFaults)
   {
      if (bFault)
      {
         //At least one dog with fault is found, set general fault value to true
         _bFault = true;
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
void RaceHandlerClass::StartTimers()
{
   _ChangeRaceState(RACING);
}

/// <summary>
///   Changes race state, if byNewRaceState is different from current one.
/// </summary>
///
/// <param name="byNewRaceState">   New race state. </param>
void RaceHandlerClass::_ChangeRaceState(RaceStates NewRaceState)
{
   //First check if the new state (this function could be called superfluously)
   if (RaceState != NewRaceState)
   {
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