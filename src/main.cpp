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

// --- Definicje zmiennych globalnych (zgodnie z config.h) ---
volatile bool measureRequested = false; // Flaga żądania pomiaru
volatile bool testRequested    = false; // Flaga żądania testu systemu
volatile bool rawRequested     = false; // Flaga żądania surowych danych

volatile unsigned long oledLastUpdateTime = 0; // Czas ostatniej aktualizacji wyświetlacza
volatile bool oledEnabled = false;              // Czy wyświetlacz ma być aktywny (domyślnie false)
volatile bool relayState  = true;              // Stan przekaźnika

// Zmienne dla logiki przycisku i pomiaru ciągłego
bool continuousMeasureEnabled = false;         // Czy tryb ciągły jest włączony
unsigned long lastContinuousMeasureTime = 0;   // Czas ostatniego pomiaru automatycznego

// Muteksy do bezpiecznej wymiany danych między rdzeniami ESP32
SemaphoreHandle_t avgArrayMutex = nullptr; 
SemaphoreHandle_t rawBufMutex   = nullptr; 

// Tablice przechowujące wyniki pomiarów
float avgArray[NUM_LIDARS];      
float stdArray[NUM_LIDARS];      
LidarBuffer lidarBuffers[NUM_LIDARS]; 

// Parametry kalibracyjne
float offsetX = 0.2f;
float offsetY = 0.2f;

// Bufor obrazu dla wyświetlacza OLED
UBYTE Image_buf[OLED_1in5_B_WIDTH * OLED_1in5_B_HEIGHT / 8];

void setup() {
  Wire.begin();           
  Serial.begin(115200);   
  delay(500);

  // --- ZMIANA 1: Przekaźnik włączony na stałe w setup ---
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Włączony od razu
  relayState = true;

  // Przycisk
  pinMode(BUTTON_PIN, INPUT_PULLDOWN); 

  // Inicjalizacja tablic
  for (uint8_t i=0; i<NUM_LIDARS; ++i){
    avgArray[i] = -1.0f;
    stdArray[i] = NAN;
    lidarBuffers[i].head = 0;
    lidarBuffers[i].count = 0;
  }

  // Muteksy
  avgArrayMutex = xSemaphoreCreateMutex();
  rawBufMutex   = xSemaphoreCreateMutex();
  if (!avgArrayMutex || !rawBufMutex) { while(1) delay(1000); }

  // OLED Init
  OLED_1in5_B_Init();
  Paint_NewImage(Image_buf, OLED_1in5_B_WIDTH, OLED_1in5_B_HEIGHT, ROTATE_0, WHITE);
  OLED_1in5_B_Display(Image_buf);
  oledLastUpdateTime = millis();

  // Inicjalizacja LIDAR
  for (uint8_t ch=1; ch<5; ++ch) {
    tcaSelect(ch); 
    int r = tf02_init_i2c_mode(); 
    Serial.print("CH"); Serial.print(ch);
    Serial.println(r==0?": OK":": FAIL");
  }

  // Taski
  xTaskCreatePinnedToCore(lidarMeasurementTask, "LidarTask", 8192, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(oledControlTask, "OledTask", 4096, nullptr, 1, nullptr, 0);

  Serial.println("Ready. Laser (Relay) is always ON.");
}

void loop() {
  // --- 1. Obsługa WSZYSTKICH poleceń JSON (Przywrócone) ---
  if (Serial.available() > 0) {
    static String buffer = "";
    char c = (char)Serial.read();
    buffer += c;

    if (buffer.indexOf('}') != -1) {
      try {
        json cmd = json::parse(buffer.c_str());
        buffer = ""; 

        if (cmd.contains("measure")) {
          measureRequested = true;
          oledLastUpdateTime = millis();
          oledEnabled = true; // Włącz ekran przy pomiarze ręcznym
          Serial.println("{\"status\":\"OK\",\"cmd\":\"measure\"}");
        } 
        else if (cmd.contains("offsetX")) {
          offsetX = cmd["offsetX"].get<float>();
          Serial.print("{\"status\":\"OK\",\"cmd\":\"offsetX\",\"value\":");
          Serial.print(offsetX, 3);
          Serial.println("}");
        } 
        else if (cmd.contains("offsetY")) {
          offsetY = cmd["offsetY"].get<float>();
          Serial.print("{\"status\":\"OK\",\"cmd\":\"offsetY\",\"value\":");
          Serial.print(offsetY, 3);
          Serial.println("}");
        } 
        else if (cmd.contains("test")) {
          testRequested = true;
          Serial.println("{\"status\":\"OK\",\"cmd\":\"test\"}");
        } 
        else if (cmd.contains("raw")) {
          rawRequested = true;
          Serial.println("{\"status\":\"OK\",\"cmd\":\"raw\"}");
        } 
        else {
          Serial.println("{\"status\":\"error\",\"reason\":\"unknown_cmd\"}");
        }
      } catch (...) {
        Serial.println("{\"status\":\"error\",\"reason\":\"json_parse\"}");
        buffer = "";
      }
    }
  }

  // --- 2. Obsługa przycisku (Toggle: Włącz/Wyłącz pomiar ciągły i Ekran) ---
  static bool lastButtonState = LOW;
  bool currentButtonState = digitalRead(BUTTON_PIN);

  if (currentButtonState == HIGH && lastButtonState == LOW) {
    continuousMeasureEnabled = !continuousMeasureEnabled; // Przełącz stan
    
    if (continuousMeasureEnabled) {
      Serial.println("[BUTTON] Pomiar ciągły: START");
      oledEnabled = true; // Włączamy ekran
    } else {
      Serial.println("[BUTTON] Pomiar ciągły: STOP");
      oledEnabled = false; // Wyłączamy ekran
    }
    
    delay(50); // Debouncing
  }
  lastButtonState = currentButtonState;

  // --- 3. Logika ciągłego pomiaru (co 2 sekundy) ---
  if (continuousMeasureEnabled) {
    // Podtrzymanie ekranu - "oszukujemy" timer wygaszacza tak długo jak tryb jest aktywny
    oledLastUpdateTime = millis(); 
    oledEnabled = true;

    // Sprawdzenie czasu co 2 sekundy
    if (millis() - lastContinuousMeasureTime >= 2000) {
      measureRequested = true; // Wyzwól pomiar
      lastContinuousMeasureTime = millis();
    }
  }

  vTaskDelay(pdMS_TO_TICKS(20));
}