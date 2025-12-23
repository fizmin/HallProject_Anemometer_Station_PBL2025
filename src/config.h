#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define TF02_I2C_ADDR 0x10
#define TCA_ADDR      0x70
#define RELAY_PIN     4
#define BUTTON_PIN    15
#define OLED_TIMEOUT_MS 10000

const uint8_t NUM_LIDARS = 5;
const uint8_t RAW_BUFFER_SIZE = 20;

struct MeasurementData {
  float dist_cm;
  uint16_t strength;
  float temp_c;
  bool valid;
};

struct LidarBuffer {
  MeasurementData buf[RAW_BUFFER_SIZE];
  uint8_t head;
  uint8_t count;
};

// globalne (deklaracje – definicje w main.cpp)
extern volatile bool measureRequested;
extern volatile bool testRequested;
extern volatile bool rawRequested;

extern volatile unsigned long oledLastUpdateTime;
extern volatile bool oledEnabled;
extern volatile bool relayState;

extern SemaphoreHandle_t avgArrayMutex;
extern SemaphoreHandle_t rawBufMutex;

extern float avgArray[NUM_LIDARS];
extern float stdArray[NUM_LIDARS];
extern LidarBuffer lidarBuffers[NUM_LIDARS];

extern float offsetX; // cm
extern float offsetY; // cm
