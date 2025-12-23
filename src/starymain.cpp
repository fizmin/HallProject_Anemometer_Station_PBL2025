#include <Arduino.h>
#include <json.hpp>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>


// ========== OLED INCLUDES ==========
#include "oled/DEV_Config.h"
#include "oled/OLED_1in5_b.h"
#include "oled/GUI_Paint.h"
#include "oled/fonts.h"


using json = nlohmann::json;


#define TF02_I2C_ADDR 0x10
#define TCA_ADDR      0x70
#define RELAY_PIN 4
#define BUTTON_PIN 15  // Przycisk podłączony do D15 (stan wysoki = 3.3V)
#define OLED_TIMEOUT_MS 10000


const uint8_t NUM_LIDARS = 5;
float avgArray[NUM_LIDARS];
UBYTE Image_buf[OLED_1in5_B_WIDTH * OLED_1in5_B_HEIGHT / 8];


volatile bool measureRequested = false;
volatile unsigned long oledLastUpdateTime = 0;
volatile bool oledEnabled = true;


// stan lasera/przekaźnika (toggle z przycisku)
volatile bool relayState = false;


SemaphoreHandle_t avgArrayMutex = nullptr;


// ========== PROTOTYPY ==========
void tcaSelect(uint8_t channel);
int tf02_read_mm_esp32();
int tf02_init_i2c_mode();
void updateOLED();
void lidarMeasurementTask(void* pv);
void oledControlTask(void* pv);


// ========== SETUP ==========
void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(500);


  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);   // Przycisk - stan wysoki = 3.3V
  relayState = false;
  digitalWrite(RELAY_PIN, LOW);


  Serial.println("\n========== SYSTEM START ==========");
  Serial.println("TF02-Pro + TCA9548A + JSON + OLED + BUTTON");


  // Utwórz mutex
  avgArrayMutex = xSemaphoreCreateMutex();
  if (avgArrayMutex == nullptr) {
    Serial.println("ERROR: Mutex creation failed");
    while (1) delay(1000);
  }


  // ========== OLED INIT ==========
  OLED_1in5_B_Init();
  Paint_NewImage(Image_buf, OLED_1in5_B_WIDTH, OLED_1in5_B_HEIGHT,
                 ROTATE_0, WHITE);
  OLED_1in5_B_Display(Image_buf);
  oledLastUpdateTime = millis();


  Serial.println("OLED initialized");


  // ========== INICJACJA LIDARÓW ==========
  for (uint8_t ch = 1; ch < 5; ++ch) {
    tcaSelect(ch);
    int r = tf02_init_i2c_mode();
    Serial.print("CH");
    Serial.print(ch);
    Serial.println(r == 0 ? ": OK" : ": FAIL");
    delay(100);
  }


  Serial.println("========== SETUP COMPLETE ==========\n");
  Serial.println("UART: {\"measure\": true} | BUTTON: D15 (toggle laser)");


  // ========== URUCHOMIENIE TASKÓW ==========
  xTaskCreatePinnedToCore(lidarMeasurementTask, "LidarTask", 8192, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(oledControlTask,  "OledTask",  4096, nullptr, 1, nullptr, 0);


  Serial.println("System gotowy");
}


// ========== MAIN LOOP ==========
void loop() {
  // 1. Obsługa UART JSON (dla Raspberry Pi)
  if (Serial.available() > 0) {
    static String buffer = "";
    char c = (char)Serial.read();
    buffer += c;


    if (buffer.indexOf('}') != -1) {
      try {
        json cmd = json::parse(buffer.c_str());
        if (cmd.contains("measure")) {
          measureRequested = true;       // Pełny pomiar z wysyłką UART
          oledLastUpdateTime = millis();
          oledEnabled = true;
          Serial.println("[OK] UART Measure + OLED on");
        }
        buffer = "";
      } catch (...) {
        if (buffer.length() > 256) buffer = "";
      }
    }
  }


  // 2. Detekcja przycisku – toggle RELAY_PIN (laser ON/OFF)
  static bool lastButtonState = LOW;
  bool currentButtonState = digitalRead(BUTTON_PIN);


  // wykrywanie zbocza narastającego (LOW -> HIGH)
  if (currentButtonState == HIGH && lastButtonState == LOW) {
    relayState = !relayState;
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
    Serial.print("[BUTTON] Relay (laser) ");
    Serial.println(relayState ? "ON" : "OFF");
  }
  lastButtonState = currentButtonState;


  vTaskDelay(pdMS_TO_TICKS(20));  // Debounce + odciążenie CPU
}


// ========== LIDAR TASK ==========
void lidarMeasurementTask(void* pv) {
  for (;;) {
    // pomiar tylko po komendzie JSON
    if (!measureRequested) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }


    measureRequested = false;
    bool doFullMeasurement = true;  // zawsze pełny pomiar z UART + OLED


    // Pomiary (wspólne)
    // Uwaga: jeśli chcesz, żeby laser był tylko na czas pomiaru,
    // to możesz tu chwilowo nadpisać RELAY_PIN, ale wtedy
    // toggle z przycisku nie będzie "trzymał" stanu sprzętowo.
    digitalWrite(RELAY_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(50));


    for (uint8_t ch = 1; ch < NUM_LIDARS; ch++) {
      tcaSelect(ch);


      long sum = 0;
      int valid_count = 0;
      int attempts = 0;


      while (valid_count < 5 && attempts < 10) {
        int d = tf02_read_mm_esp32();
        attempts++;
        if (d >= 0) {
          sum += d;
          valid_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
      }


      xSemaphoreTake(avgArrayMutex, portMAX_DELAY);
      {
        if (valid_count > 0) {
          avgArray[ch] = (float)sum / (float)valid_count / 10.0f;
        } else {
          avgArray[ch] = -1.0f;
        }
      }
      xSemaphoreGive(avgArrayMutex);
    }


    digitalWrite(RELAY_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(50));


    // Wyślij JSON TYLKO dla UART (Raspberry Pi)
    if (doFullMeasurement) {
      xSemaphoreTake(avgArrayMutex, portMAX_DELAY);
      {
        // Para 1 (LiDAR 1 + 2)
        float avg_pair1 = 0;
        String instruction_pair1 = "";
        int valid_pair1 = 0;


        if (avgArray[1] >= 0 && avgArray[2] >= 0) {
          avg_pair1 = (avgArray[1] + avgArray[2]) / 2.0f;
          float diff = avgArray[1] - avgArray[2];
          if (diff > 6) {
            instruction_pair1 = "Obrot CW";
          } else if (diff < -6) {
            instruction_pair1 = "Obrot CCW";
          } else {
            instruction_pair1 = "Lidar OK";
          }
          valid_pair1 = 2;
        } else if (avgArray[1] >= 0) {
          avg_pair1 = avgArray[1];
          instruction_pair1 = "Lidar 2 error";
          valid_pair1 = 1;
        } else if (avgArray[2] >= 0) {
          avg_pair1 = avgArray[2];
          instruction_pair1 = "Lidar 1 error";
          valid_pair1 = 1;
        } else {
          instruction_pair1 = "Both error";
          valid_pair1 = 0;
        }


        // Para 2 (LiDAR 3 + 4)
        float avg_pair2 = 0;
        String instruction_pair2 = "";
        int valid_pair2 = 0;


        if (avgArray[3] >= 0 && avgArray[4] >= 0) {
          avg_pair2 = (avgArray[3] + avgArray[4]) / 2.0f;
          float diff = avgArray[3] - avgArray[4];
          if (diff > 6) {
            instruction_pair2 = "Obrot CW";
          } else if (diff < -6) {
            instruction_pair2 = "Obrot CCW";
          } else {
            instruction_pair2 = "Lidar OK";
          }
          valid_pair2 = 2;
        } else if (avgArray[3] >= 0) {
          avg_pair2 = avgArray[3];
          instruction_pair2 = "Lidar 4 error";
          valid_pair2 = 1;
        } else if (avgArray[4] >= 0) {
          avg_pair2 = avgArray[4];
          instruction_pair2 = "Lidar 3 error";
          valid_pair2 = 1;
        } else {
          instruction_pair2 = "Both error";
          valid_pair2 = 0;
        }


        // JSON 1: Para 1
        json pair1;
        pair1["pair"] = "1 (Lidar 1+2)";
        pair1["average"] = (valid_pair1 > 0) ? avg_pair1 : -1.0f;
        pair1["instruction"] = instruction_pair1.c_str();
        Serial.println(pair1.dump().c_str());


        // JSON 2: Para 2
        json pair2;
        pair2["pair"] = "2 (Lidar 3+4)";
        pair2["average"] = (valid_pair2 > 0) ? avg_pair2 : -1.0f;
        pair2["instruction"] = instruction_pair2.c_str();
        Serial.println(pair2.dump().c_str());


        // JSON 3: Wszystkie 4 lidary
        json all;
        all["lidar_1"] = avgArray[1];
        all["lidar_2"] = avgArray[2];
        all["lidar_3"] = avgArray[3];
        all["lidar_4"] = avgArray[4];
        Serial.println(all.dump().c_str());
      }
      xSemaphoreGive(avgArrayMutex);
      Serial.println("[OK] UART Measure complete");
    }
  }
}


// ========== OLED TASK ==========
void oledControlTask(void* pv) {
  for (;;) {
    unsigned long now = millis();
    unsigned long timeSinceUpdate = now - oledLastUpdateTime;


    if (oledEnabled && timeSinceUpdate >= OLED_TIMEOUT_MS) {
      xSemaphoreTake(avgArrayMutex, pdMS_TO_TICKS(100));
      {
        Paint_DrawRectangle(0, 0, 128, 128, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        OLED_1in5_B_Display(Image_buf);
      }
      xSemaphoreGive(avgArrayMutex);
      oledEnabled = false;
      Serial.println("[OLED] Timeout - screen off");
    }


    if (oledEnabled) {
      updateOLED();
    }


    vTaskDelay(pdMS_TO_TICKS(500));
  }
}


// ========== WYŚWIETLANIE OLED ==========
void updateOLED() {
  if (!xSemaphoreTake(avgArrayMutex, pdMS_TO_TICKS(100))) {
    return;
  }


  Paint_DrawRectangle(0, 0, 128, 128, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);


  // Oś X (Para 1: LiDAR 1+2)
  Paint_DrawString_EN(5, 3, "Os X", &Font16, BLACK, WHITE);


  float avg_x = 0;
  String instr_x = "";
  int valid_x = 0;


  if (avgArray[1] >= 0 && avgArray[2] >= 0) {
    avg_x = (avgArray[1] + avgArray[2]) / 2.0f;
    float diff = avgArray[1] - avgArray[2];
    if (diff > 10) instr_x = "Obr CW";
    else if (diff < -10) instr_x = "Obr CCW";
    else instr_x = "OK";
    valid_x = 2;
  } else if (avgArray[1] >= 0) {
    avg_x = avgArray[1];
    instr_x = "L2 err";
    valid_x = 1;
  } else if (avgArray[2] >= 0) {
    avg_x = avgArray[2];
    instr_x = "L1 err";
    valid_x = 1;
  } else {
    instr_x = "Both err";
  }


  char buf_x[16];
  if (valid_x > 0) {
    sprintf(buf_x, "%.1f cm", avg_x);
  } else {
    sprintf(buf_x, "ERROR");
  }
  Paint_DrawString_EN(5, 20, buf_x, &Font16, BLACK, WHITE);
  Paint_DrawString_EN(5, 36, instr_x.c_str(), &Font16, BLACK, WHITE);


  Paint_DrawLine(0, 54, 127, 54, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);


  // Oś Y (Para 2: LiDAR 3+4)
  Paint_DrawString_EN(5, 58, "Os Y", &Font16, BLACK, WHITE);


  float avg_y = 0;
  String instr_y = "";
  int valid_y = 0;


  if (avgArray[3] >= 0 && avgArray[4] >= 0) {
    avg_y = (avgArray[3] + avgArray[4]) / 2.0f;
    float diff = avgArray[3] - avgArray[4];
    if (diff > 10) instr_y = "Obr CW";
    else if (diff < -10) instr_y = "Obr CCW";
    else instr_y = "OK";
    valid_y = 2;
  } else if (avgArray[3] >= 0) {
    avg_y = avgArray[3];
    instr_y = "L4 err";
    valid_y = 1;
  } else if (avgArray[4] >= 0) {
    avg_y = avgArray[4];
    instr_y = "L3 err";
    valid_y = 1;
  } else {
    instr_y = "Both err";
  }


  char buf_y[16];
  if (valid_y > 0) {
    sprintf(buf_y, "%.1f cm", avg_y);
  } else {
    sprintf(buf_y, "ERROR");
  }
  Paint_DrawString_EN(5, 75, buf_y, &Font16, BLACK, WHITE);
  Paint_DrawString_EN(5, 91, instr_y.c_str(), &Font16, BLACK, WHITE);


  OLED_1in5_B_Display(Image_buf);


  xSemaphoreGive(avgArrayMutex);
}


// ========== FUNKCJE POMOCNICZE ==========
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}


int tf02_read_mm_esp32() {
  Wire.beginTransmission(TF02_I2C_ADDR);
  Wire.write(0x5A); Wire.write(0x05); Wire.write(0x00);
  Wire.write(0x06); Wire.write(0x65);
  if (Wire.endTransmission() != 0) return -1;


  uint8_t frame[9];
  if (Wire.requestFrom(TF02_I2C_ADDR, 9) != 9) return -2;


  for (int i = 0; i < 9; ++i) {
    if (!Wire.available()) return -3;
    frame[i] = Wire.read();
  }


  if (frame[0] != 0x59 || frame[1] != 0x59) return -4;


  uint8_t sum = 0;
  for (int i = 0; i < 8; ++i) sum += frame[i];
  if (sum != frame[8]) return -5;


  uint16_t dist_mm = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);
  if (dist_mm == 45000) return -6;


  return (int)dist_mm;
}


int tf02_init_i2c_mode() {
  Wire.beginTransmission(TF02_I2C_ADDR);
  Wire.write(0xAA); Wire.write(0x55); Wire.write(0xF0); Wire.write(0x00);
  Wire.write(0x01); Wire.write(0x00); Wire.write(0x00); Wire.write(0x02);
  if (Wire.endTransmission() != 0) return -1;
  vTaskDelay(pdMS_TO_TICKS(100));


  Wire.beginTransmission(TF02_I2C_ADDR);
  Wire.write(0x5A); Wire.write(0x05); Wire.write(0x0A); Wire.write(0x01); Wire.write(0x6A);
  if (Wire.endTransmission() != 0) return -2;
  vTaskDelay(pdMS_TO_TICKS(100));


  Wire.beginTransmission(TF02_I2C_ADDR);
  Wire.write(0x5A); Wire.write(0x04); Wire.write(0x11); Wire.write(0x6F);
  if (Wire.endTransmission() != 0) return -3;
  vTaskDelay(pdMS_TO_TICKS(100));


  Wire.beginTransmission(TF02_I2C_ADDR);
  Wire.write(0xAA); Wire.write(0x55); Wire.write(0xF0); Wire.write(0x00);
  Wire.write(0x00); Wire.write(0x00); Wire.write(0x00); Wire.write(0x02);
  if (Wire.endTransmission() != 0) return -4;
  vTaskDelay(pdMS_TO_TICKS(100));


  Wire.beginTransmission(TF02_I2C_ADDR);
  Wire.write(0xAA); Wire.write(0x55); Wire.write(0xF0); Wire.write(0x00);
  Wire.write(0xFF); Wire.write(0xFF); Wire.write(0xFF); Wire.write(0xFF);
  if (Wire.endTransmission() != 0) return -5;
  vTaskDelay(pdMS_TO_TICKS(200));


  return 0;
}