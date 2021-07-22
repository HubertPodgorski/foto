#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include "Structs.h"
#include <RaceHandler.h>
#include <LightsController.h>
#include <LCDController.h>

LiquidCrystal_I2C lcd(0x27,20,4);

#define SENSOR_1_PIN 3
#define SENSOR_2_PIN 2

#define LIGHT_PIN_1 4
#define LIGHT_PIN_2 11
#define LIGHT_PIN_3 12
#define LIGHT_PIN_4 5

#define BUTTON_PIN 7

long startTime; // When Sensor 1 is triggered
long elapsedTime;
long lastTriggerTime;
volatile long dogExitTriggerTime;
volatile long dogEnterTriggerTime;
long lastLightsTriggerTime;
long dogEnterTime;
long dogLapTime;
unsigned long lastButtonPressTime;

volatile int sensorFrontState; // Front sensor status
volatile int boxSensorState; // Front sensor status

unsigned long msButtonDelay = 300;

bool earlyEnter = false;
bool showingResults = false;
bool showingWaitingMessage = false;
bool light1On = false;
bool light2On = false;
bool light3On = false;
bool light4On = false;

void CleanupAllData();
void DisplayResults();
char * TimeToString(unsigned long givenMsTime);
void CalculateResults();
void CheckButtonTrigger();
void DisplayRaceTime();
void CheckTurnOnLights();
void TurnOffAllLights();
void StartStopRace();
void ResetRace();


uint8_t CurrentDog;
uint8_t CurrentRaceState;

char DogTime[8];
char DogCrossingTime[8];
char ElapsedRaceTime[8];
char TotalCrossingTime[8];

void Sensor1Wrapper();
void Sensor2Wrapper();

void setup() {
  Serial.begin(9600);

  pinMode(LIGHT_PIN_1, OUTPUT);
  pinMode(LIGHT_PIN_2, OUTPUT);
  pinMode(LIGHT_PIN_3, OUTPUT);
  pinMode(LIGHT_PIN_4, OUTPUT);

  lcd.backlight();
  lcd.init();

  LCDController.init(&lcd);

  CleanupAllData();

  attachInterrupt(digitalPinToInterrupt(SENSOR_1_PIN), Sensor1Wrapper, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SENSOR_2_PIN), Sensor2Wrapper, CHANGE);

  LightsController.Init(LIGHT_PIN_1, LIGHT_PIN_2, LIGHT_PIN_3, LIGHT_PIN_4);
}

void loop() {
  /*
   Handle lights and start timer on green
  */
  LightsController.Main();

  //Handle Race main processing
  RaceHandler.Main();

  //Handle LCD processing
  LCDController.Main();

  /* 
   *  Checking button trigger action and determining state
  */
  CheckButtonTrigger();

  //Update LCD Display fields
  //Update team time to display
  dtostrf(RaceHandler.GetRaceTime(), 7, 3, ElapsedRaceTime);
  LCDController.UpdateField(LCDController.TeamTime, ElapsedRaceTime);

  //Update total crossing time
   dtostrf(RaceHandler.GetTotalCrossingTime(), 7, 3, TotalCrossingTime);
   LCDController.UpdateField(LCDController.TotalCrossTime, TotalCrossingTime);

   //Update race status to display
   LCDController.UpdateField(LCDController.RaceState, RaceHandler.GetRaceStateString());

   //Handle individual dog info
   dtostrf(RaceHandler.GetDogTime(0), 7, 3, DogTime);
   LCDController.UpdateField(LCDController.D1Time, DogTime);
   LCDController.UpdateField(LCDController.D1CrossTime, RaceHandler.GetCrossingTime(0));
   LCDController.UpdateField(LCDController.D1RerunInfo, RaceHandler.GetRerunInfo(0));

   dtostrf(RaceHandler.GetDogTime(1), 7, 3, DogTime);
   LCDController.UpdateField(LCDController.D2Time, DogTime);
   LCDController.UpdateField(LCDController.D2CrossTime, RaceHandler.GetCrossingTime(1));
   LCDController.UpdateField(LCDController.D2RerunInfo, RaceHandler.GetRerunInfo(1));

   dtostrf(RaceHandler.GetDogTime(2), 7, 3, DogTime);
   LCDController.UpdateField(LCDController.D3Time, DogTime);
   LCDController.UpdateField(LCDController.D3CrossTime, RaceHandler.GetCrossingTime(2));
   LCDController.UpdateField(LCDController.D3RerunInfo, RaceHandler.GetRerunInfo(2));

   dtostrf(RaceHandler.GetDogTime(3), 7, 3, DogTime);
   LCDController.UpdateField(LCDController.D4Time, DogTime);
   LCDController.UpdateField(LCDController.D4CrossTime, RaceHandler.GetCrossingTime(3));
   LCDController.UpdateField(LCDController.D4RerunInfo, RaceHandler.GetRerunInfo(3));

  if (CurrentRaceState != RaceHandler.RaceState) {
    // TODO: do logging
      if (RaceHandler.RaceState == RaceHandler.STOP) {
         //Race is finished, put final data on screen
        //  dtostrf(RaceHandler.GetDogTime(RaceHandler.CurrentDog, -2), 7, 3, DogTime);
        //  ESP_LOGI(__FILE__, "D%i: %s|CR: %s", RaceHandler.CurrentDog, DogTime, RaceHandler.GetCrossingTime(RaceHandler.CurrentDog, -2).c_str());
        //  ESP_LOGI(__FILE__, "RT:%s", ElapsedRaceTime);
      }
      // ESP_LOGI(__FILE__, "RS: %i", RaceHandler.RaceState);
   }

   if (RaceHandler.CurrentDog != CurrentDog) {
    //  TODO: do logging
      // dtostrf(RaceHandler.GetDogTime(RaceHandler.iPreviousDog, -2), 7, 3, DogTime);
      // ESP_LOGI(__FILE__, "D%i: %s|CR: %s", RaceHandler.iPreviousDog, DogTime, RaceHandler.GetCrossingTime(RaceHandler.iPreviousDog, -2).c_str());
      // ESP_LOGI(__FILE__, "D: %i", RaceHandler.CurrentDog);
      // ESP_LOGI(__FILE__, "RT:%s", ElapsedRaceTime);
   }

   //Cleanup variables used for checking if something changed
   CurrentDog = RaceHandler.CurrentDog;
   CurrentRaceState = RaceHandler.RaceState;
}

void CheckButtonTrigger() {
  int buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == HIGH && (millis() - lastButtonPressTime >= msButtonDelay)) {
    StartStopRace();
  };
}

/// <summary>
///   Starts (if stopped) or stops (if started) a race. Start is only allowed if race is stopped and reset.
/// </summary>
void StartStopRace() {
   lastButtonPressTime = millis();
  //If race is stopped and timers are zero
  if (RaceHandler.RaceState == RaceHandler.STOP && RaceHandler.GetRaceTime() == 0) {
      //Then start the race
      // ESP_LOGD(__FILE__, "%lu: START!", millis());
      LightsController.InitiateStartSequence();
      RaceHandler.StartRace();
   } else if (RaceHandler.RaceState == RaceHandler.STOP && RaceHandler.GetRaceTime() != 0) {
     ResetRace();
   } else {
      RaceHandler.StopRace();
      LightsController.DeleteSchedules();
   }
}

/// <summary>
///   Reset race so new one can be started, reset is only allowed when race is stopped
/// </summary>
void ResetRace() {
   if (RaceHandler.RaceState != RaceHandler.STOP) {
      return;
   }

  //  TODO: 
  // <<<<<<>>>>>>>>>>
   LightsController.ResetLights();
   RaceHandler.ResetRace();
  // <<<<<<>>>>>>>>>>
}

char * TimeToString(unsigned long givenMsTime) {
  static char str[9];

  int seconds = givenMsTime / 1000;
  int ms = givenMsTime % 1000;

  sprintf(str, "%01ds %02dms", seconds, ms);

  return str;
}

void Sensor2Wrapper()
{
   RaceHandler.TriggerSensor2();
}

void Sensor1Wrapper()
{
   RaceHandler.TriggerSensor1();
}