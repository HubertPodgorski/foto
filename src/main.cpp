#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);

#define FRONT_SENSOR_PIN 3
#define BOX_SENSOR_PIN 2

#define LIGHT_PIN_1 4
#define LIGHT_PIN_2 11
#define LIGHT_PIN_3 12
#define LIGHT_PIN_4 5

#define BUTTON_PIN 7


//#define FRONT_SENSOR_LED_PIN A0

long startTime; // When Sensor 1 is triggered
long elapsedTime;
long lastTriggerTime;
volatile long dogExitTriggerTime;
volatile long dogEnterTriggerTime;
long lastLightsTriggerTime;
long dogEnterTime;
long dogLapTime;
long lastButtonPressTime;

volatile int sensorFrontState; // Front sensor status
volatile int boxSensorState; // Front sensor status

int msBetweenLights = 1000;
int msDelayDogLap = 1000;
int msButtonDelay = 300;

bool earlyEnter = false;
bool showingResults = false;
bool showingWaitingMessage = false;
bool light1On = false;
bool light2On = false;
bool light3On = false;
bool light4On = false;

enum AppState {
  RACING,
  STOP
};

AppState appState = STOP;

void CleanupAllData();
void DisplayResults();
char * TimeToString(unsigned long givenMsTime);
void CalculateResults();
void CheckButtonTrigger();
void DisplayRaceTime();
void CheckTurnOnLights();
void TurnOffAllLights();

void FrontSensorBroken() {
  if(appState != RACING) {
    return;
  }

  sensorFrontState = digitalRead(FRONT_SENSOR_PIN);
  Serial.println(sensorFrontState);

  if (sensorFrontState != HIGH) {
    return;
  }

  if (!dogEnterTriggerTime) {
    dogEnterTriggerTime = millis();
    
    return;
  }

  if (!dogExitTriggerTime && millis() - dogEnterTriggerTime > msDelayDogLap ) {
    dogExitTriggerTime = millis();
  }
}

void BoxSensorBroken() {
  if(appState != RACING) {
    return;
  }

  boxSensorState = digitalRead(BOX_SENSOR_PIN);
  Serial.print("boxSensorState => ");
  Serial.println(boxSensorState);
}

void setup() {
  Serial.begin(9600);

  pinMode(LIGHT_PIN_1, OUTPUT);
  pinMode(LIGHT_PIN_2, OUTPUT);
  pinMode(LIGHT_PIN_3, OUTPUT);
  pinMode(LIGHT_PIN_4, OUTPUT);

  lcd.backlight();
  lcd.init();

  CleanupAllData();

  attachInterrupt(digitalPinToInterrupt(FRONT_SENSOR_PIN), FrontSensorBroken, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BOX_SENSOR_PIN), BoxSensorBroken, CHANGE);
}

void loop() {
  /* 
   *  Checking button trigger action and determining state
  */
  CheckButtonTrigger();

  /* 
   *  Handle app state - RACING - dog is running, lights are turned on
  */
  if (appState == RACING) {
    DisplayRaceTime();

    /* checking light to turn on */
    if (!lastLightsTriggerTime) {
      CheckTurnOnLights();
    }
  }

  /* JUST BEFORE CROSSES COME */
  if (dogEnterTriggerTime && dogExitTriggerTime) {
    appState = STOP;
  }

  /* 
   *  Handle app state - RESULTS - app is displaying result
  */
  if (appState == STOP) {
    if ((dogEnterTriggerTime && !dogEnterTime) || (dogExitTriggerTime && !dogLapTime)) {
      CalculateResults();
    }

    
    if (!showingResults) {
      DisplayResults();
    }
  }
}

void CheckButtonTrigger() {
  int buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == LOW || (millis() - lastButtonPressTime < msButtonDelay)) {
    return;
  }

  lastButtonPressTime = millis();

  if (appState == RACING) {
    appState = STOP;
  } else {
    CleanupAllData();
    
    appState = RACING;
  }
}

void DisplayRaceTime() {
  /* needed? - showing time on display - move to method */
  if (lastLightsTriggerTime && !showingResults) {
    lcd.setCursor(0, 0);
    lcd.print("Czas: ");
    lcd.print(TimeToString(millis() - lastLightsTriggerTime));
  }
}

void CleanupAllData() {
  lcd.clear();
  startTime = millis();
  dogEnterTriggerTime = 0;
  dogExitTriggerTime = 0;
  showingResults = false;
  earlyEnter = false;
  dogEnterTime = 0;
  dogLapTime = 0;
  lastLightsTriggerTime = 0;
  lastTriggerTime = 0;
  showingWaitingMessage = false;
  TurnOffAllLights();
}

void DisplayResults() {
  lcd.clear();

  if (dogEnterTime) {
    lcd.setCursor(0, 0);
    lcd.print("Wej");
    if (earlyEnter) {
       lcd.print(" [E]");  
    }
    lcd.print(": ");
  
    lcd.print(TimeToString(dogEnterTime));
  }

  if (dogLapTime) {
    lcd.setCursor(0, 1);
    lcd.print("Bieg: ");
    lcd.print(TimeToString(dogLapTime));
  }

  if (!dogEnterTime && !dogLapTime) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Przerwano");
  }

  showingResults = true;
}

void CalculateResults() {
  if (dogEnterTriggerTime && lastLightsTriggerTime) {
    if (dogEnterTriggerTime > lastLightsTriggerTime) {
      dogEnterTime = dogEnterTriggerTime - lastLightsTriggerTime;
    } else {
      earlyEnter = true;
      dogEnterTime = lastLightsTriggerTime - dogEnterTriggerTime;
    }
  }

  if (dogExitTriggerTime && dogEnterTriggerTime) {
    dogLapTime = dogExitTriggerTime - dogEnterTriggerTime;
  }

    Serial.println("<=======>");

     Serial.print("dogEnterTime => ");
     Serial.println(dogEnterTime);

     Serial.print("dogLapTime => ");
     Serial.println(dogLapTime);

     
     Serial.println("<=======>");
}

void CheckTurnOnLights() {
  if (!light1On) {
      lastTriggerTime = millis();

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("3!");
      
      Serial.println("turn on 1");

      digitalWrite(LIGHT_PIN_1, HIGH);

      light1On = true;
    }

    if (!light2On && light1On) {
      // TODO: between lights time
      if (millis() - lastTriggerTime >= msBetweenLights) {
        lastTriggerTime = millis();

        Serial.println("turn on 2");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("2!");

        digitalWrite(LIGHT_PIN_1, LOW);
        digitalWrite(LIGHT_PIN_2, HIGH);

        light2On = true;
      }
    }

    if (!light3On && light2On) {
      // TODO: between lights time
      if (millis() - lastTriggerTime >= msBetweenLights) {
        lastTriggerTime = millis();

        Serial.println("turn on 3");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("1!");

        digitalWrite(LIGHT_PIN_2, LOW);
        digitalWrite(LIGHT_PIN_3, HIGH);
      
        light3On = true;
      }
    }

    if (!light4On && light3On) {
      // TODO: between lights time
      if (millis() - lastTriggerTime >= msBetweenLights) {
        lastTriggerTime = millis();
        lastLightsTriggerTime = millis();

        Serial.println("turn on 4");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("START");

        digitalWrite(LIGHT_PIN_3, LOW);
        digitalWrite(LIGHT_PIN_4, HIGH);

        light4On = true;
      } 
    }
}

void TurnOffAllLights() {
  digitalWrite(LIGHT_PIN_1, LOW);
  digitalWrite(LIGHT_PIN_2, LOW);
  digitalWrite(LIGHT_PIN_3, LOW);
  digitalWrite(LIGHT_PIN_4, LOW);

  light1On = false;
  light2On = false;
  light3On = false;
  light4On = false;
}
  

char * TimeToString(unsigned long givenMsTime) {
  static char str[9];

  int seconds = givenMsTime / 1000;
  int ms = givenMsTime % 1000;

  sprintf(str, "%01ds %02dms", seconds, ms);

  return str;
}