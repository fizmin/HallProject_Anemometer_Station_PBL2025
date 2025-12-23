#include <Arduino.h>
#include <Wire.h>
#include <json.hpp>
#include "config.h"
#include "lidar.h"
#include "measurement.h"
#include "display.h"
#include "oled/DEV_Config.h"
#include "oled/OLED_1in5_b.h"
#include "oled/GUI_Paint.h"
#include "oled/fonts.h"

using json = nlohmann::json;

// definicje globalnych z config.h
volatile bool measureRequested = false;
volatile bool testRequested    = false;
volatile bool rawRequested     = false;

volatile unsigned long oledLastUpdateTime = 0;
volatile bool oledEnabled = true;
volatile bool relayState  = false;

SemaphoreHandle_t avgArrayMutex = nullptr;
SemaphoreHandle_t rawBufMutex  = nullptr;

float avgArray[NUM_LIDARS];
float stdArray[NUM_LIDARS];
LidarBuffer lidarBuffers[NUM_LIDARS];

float offsetX = 0.2f;
float offsetY = 0.2f;

UBYTE Image_buf[OLED_1in5_B_WIDTH * OLED_1in5_B_HEIGHT / 8];

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(500);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  relayState = false;
  digitalWrite(RELAY_PIN, LOW);

  for (uint8_t i=0;i<NUM_LIDARS;++i){
    avgArray[i] = -1.0f;
    stdArray[i] = NAN;
    lidarBuffers[i].head = 0;
    lidarBuffers[i].count = 0;
  }

  avgArrayMutex = xSemaphoreCreateMutex();
  rawBufMutex   = xSemaphoreCreateMutex();
  if (!avgArrayMutex || !rawBufMutex) { while(1) delay(1000); }

  OLED_1in5_B_Init();
  Paint_NewImage(Image_buf, OLED_1in5_B_WIDTH, OLED_1in5_B_HEIGHT, ROTATE_0, WHITE);
  OLED_1in5_B_Display(Image_buf);
  oledLastUpdateTime = millis();

  for (uint8_t ch=1; ch<5; ++ch) {
    tcaSelect(ch);
    int r = tf02_init_i2c_mode();
    Serial.print("CH"); Serial.print(ch);
    Serial.println(r==0?": OK":": FAIL");
  }

  xTaskCreatePinnedToCore(lidarMeasurementTask,"LidarTask",8192,nullptr,2,nullptr,1);
  xTaskCreatePinnedToCore(oledControlTask,"OledTask",4096,nullptr,1,nullptr,0);

  Serial.println("Ready");
}

void loop() {
  // 1. Obsługa UART JSON
  if (Serial.available() > 0) {
    static String buffer = "";
    char c = (char)Serial.read();
    buffer += c;

    if (buffer.indexOf('}') != -1) {   // bardzo prosty detektor końca
      try {
        json cmd = json::parse(buffer.c_str());
        buffer = "";

        if (cmd.contains("measure")) {
          measureRequested = true;
          oledLastUpdateTime = millis();
          oledEnabled = true;
          Serial.println("{\"status\":\"OK\",\"cmd\":\"measure\"}");
        } else if (cmd.contains("offsetX")) {
          offsetX = cmd["offsetX"].get<float>();
          Serial.print("{\"status\":\"OK\",\"cmd\":\"offsetX\",\"value\":");
          Serial.print(offsetX, 3);
          Serial.println("}");
        } else if (cmd.contains("offsetY")) {
          offsetY = cmd["offsetY"].get<float>();
          Serial.print("{\"status\":\"OK\",\"cmd\":\"offsetY\",\"value\":");
          Serial.print(offsetY, 3);
          Serial.println("}");
        } else if (cmd.contains("test")) {
          testRequested = true;
          Serial.println("{\"status\":\"OK\",\"cmd\":\"test\"}");
        } else if (cmd.contains("raw")) {
          rawRequested = true;
          Serial.println("{\"status\":\"OK\",\"cmd\":\"raw\"}");
        } else {
          Serial.println("{\"status\":\"error\",\"reason\":\"unknown_cmd\"}");
        }
      } catch (...) {
        Serial.println("{\"status\":\"error\",\"reason\":\"json_parse\"}");
        buffer = "";
      }
    }
  }

  // 2. Przycisk – toggle RELAY_PIN
  static bool lastButtonState = LOW;
  bool currentButtonState = digitalRead(BUTTON_PIN);

  if (currentButtonState == HIGH && lastButtonState == LOW) {
    relayState = !relayState;
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
    Serial.print("[BUTTON] Relay (laser) ");
    Serial.println(relayState ? "ON" : "OFF");
  }
  lastButtonState = currentButtonState;

  vTaskDelay(pdMS_TO_TICKS(20));
}