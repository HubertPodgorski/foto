#pragma once
// #include <rom/rtc.h>

struct DogTimeData {
   unsigned long Time;
   long CrossingTime;
};

struct stDogData {
   uint8_t DogNumber;
   //String DogName;
   DogTimeData Timing[4];
   boolean Running;
   boolean Fault;
};

struct RaceData {
   unsigned int Id;
   unsigned long StartTime;
   unsigned long EndTime;
   unsigned long ElapsedTime;
   uint8_t RaceState;
   stDogData DogData[4];
   long TotalCrossingTime;
};