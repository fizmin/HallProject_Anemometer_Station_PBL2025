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

// --- Zmienne globalne sterujące stanem systemu ---
volatile bool measureRequested = false; // Flaga: czy żądany jest pojedynczy pomiar
volatile bool testRequested     = false; // Flaga: czy uruchomić test systemu
volatile bool rawRequested      = false; // Flaga: czy wysłać surowe dane przez Serial

volatile unsigned long oledLastUpdateTime = 0; // Czas ostatniego odświeżenia ekranu
volatile bool oledEnabled = true;               // Czy wyświetlacz jest włączony

// --- Nowe zmienne dla logiki przycisku i pomiarów automatycznych ---
// Flaga określająca, czy tryb ciągłego pomiaru (co 2 sekundy) jest aktywny
bool continuousMeasureEnabled = false;   
// Przechowuje czas (w ms) ostatniego automatycznego wyzwolenia pomiaru
unsigned long lastContinuousMeasureTime = 0; 

// Muteksy do bezpiecznej komunikacji między rdzeniami ESP32
SemaphoreHandle_t avgArrayMutex = nullptr; 
SemaphoreHandle_t rawBufMutex   = nullptr; 

// Struktury przechowujące wyniki z sensorów LIDAR
float avgArray[NUM_LIDARS]; 
float stdArray[NUM_LIDARS]; 
LidarBuffer lidarBuffers[NUM_LIDARS]; 

// Parametry geometryczne anemometru (kalibracja)
float offsetX = 0.2f;
float offsetY = 0.2f;

// Bufor graficzny dla wyświetlacza OLED
UBYTE Image_buf[OLED_1in5_B_WIDTH * OLED_1in5_B_HEIGHT / 8];

void setup() {
  Wire.begin();           // Start magistrali I2C (komunikacja z sensorami i ekranem)
  Serial.begin(115200);   // Start portu szeregowego
  delay(500);

  // --- KONFIGURACJA PINÓW ---
  // Zgodnie z wymaganiem: Przekaźnik włączony na stałe od startu
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); 
  
  // Przycisk skonfigurowany z rezystorem ściągającym do masy (INPUT_PULLDOWN)
  pinMode(BUTTON_PIN, INPUT_PULLDOWN); 

  // Inicjalizacja tablic pomiarowych wartościami początkowymi
  for (uint8_t i=0; i<NUM_LIDARS; ++i){
    avgArray[i] = -1.0f;
    stdArray[i] = NAN;
    lidarBuffers[i].head = 0;
    lidarBuffers[i].count = 0;
  }

  // Tworzenie muteksów - zapobiegają błędom przy jednoczesnym dostępie do danych z dwóch rdzeni
  avgArrayMutex = xSemaphoreCreateMutex();
  rawBufMutex   = xSemaphoreCreateMutex();
  if (!avgArrayMutex || !rawBufMutex) { while(1) delay(1000); }

  // Inicjalizacja sterownika ekranu i wyczyszczenie wyświetlacza
  OLED_1in5_B_Init();
  Paint_NewImage(Image_buf, OLED_1in5_B_WIDTH, OLED_1in5_B_HEIGHT, ROTATE_0, WHITE);
  OLED_1in5_B_Display(Image_buf);
  oledLastUpdateTime = millis();

  // Inicjalizacja sensorów LIDAR (kanały 1-4 na multiplekserze TCA)
  for (uint8_t ch=1; ch<5; ++ch) {
    tcaSelect(ch); 
    int r = tf02_init_i2c_mode(); 
    Serial.print("Sensor CH"); Serial.print(ch);
    Serial.println(r==0?": OK":": Blad");
  }

  // --- URUCHOMIENIE ZADAŃ WIELOWĄTKOWYCH (FreeRTOS) ---
  // Zadanie pomiarowe na rdzeniu 1 (priorytetowe)
  xTaskCreatePinnedToCore(lidarMeasurementTask, "LidarTask", 8192, nullptr, 2, nullptr, 1);
  // Zadanie obsługi ekranu na rdzeniu 0
  xTaskCreatePinnedToCore(oledControlTask, "OledTask", 4096, nullptr, 1, nullptr, 0);

  Serial.println("System gotowy. Przekaznik: ON (na stale).");
}

void loop() {
  // --- 1. Obsługa komend przychodzących przez Serial (format JSON) ---
  if (Serial.available() > 0) {
    static String buffer = "";
    char c = (char)Serial.read();
    buffer += c;

    // Jeśli wykryto znak kończący obiekt JSON
    if (buffer.indexOf('}') != -1) {
      try {
        json cmd = json::parse(buffer.c_str());
        buffer = ""; // Czyszczenie bufora

        if (cmd.contains("measure")) {
          measureRequested = true;
          oledLastUpdateTime = millis();
          oledEnabled = true;
          Serial.println("{\"status\":\"OK\",\"cmd\":\"measure\"}");
        } 
        else if (cmd.contains("offsetX")) {
          offsetX = cmd["offsetX"].get<float>();
          Serial.println("{\"status\":\"OK\",\"cmd\":\"offsetX\"}");
        } 
        else if (cmd.contains("test")) {
          testRequested = true;
          Serial.println("{\"status\":\"OK\",\"cmd\":\"test\"}");
        }
      } catch (...) {
        Serial.println("{\"status\":\"error\",\"reason\":\"blad_parsowania_json\"}");
        buffer = "";
      }
    }
  }

  // --- 2. Logika przycisku (Tryb Toggle / Przełącznik) ---
  static bool lastButtonState = LOW;
  bool currentButtonState = digitalRead(BUTTON_PIN);

  // Wykrycie momentu naciśnięcia (zbocze narastające)
  if (currentButtonState == HIGH && lastButtonState == LOW) {
    // Odwrócenie stanu ciągłego pomiaru
    continuousMeasureEnabled = !continuousMeasureEnabled; 
    
    Serial.print("[PRZYCISK] Pomiar ciagly: ");
    Serial.println(continuousMeasureEnabled ? "WLACZONY" : "WYLACZONY");
    
    // Wymuszenie odświeżenia ekranu przy zmianie trybu
    oledLastUpdateTime = millis();
    oledEnabled = true;

    // Krótki delay dla eliminacji drgań styków (debouncing)
    delay(50); 
  }
  lastButtonState = currentButtonState;

  // --- 3. Mechanizm pomiaru co 2 sekundy ---
  // Jeśli użytkownik włączył tryb ciągły przyciskiem
  if (continuousMeasureEnabled) {
    unsigned long currentMillis = millis();
    // Sprawdzenie, czy upłynęły 2 sekundy od ostatniego wyzwolenia
    if (currentMillis - lastContinuousMeasureTime >= 2000) {
      measureRequested = true; // Ustawienie flagi dla zadania pomiarowego
      lastContinuousMeasureTime = currentMillis; // Zapisanie czasu ostatniego pomiaru
      Serial.println("[AUTO] Wyzwolono pomiar automatyczny (interwal 2s)");
    }
  }

  // Oddanie czasu procesora dla innych zadań w systemie FreeRTOS
  vTaskDelay(pdMS_TO_TICKS(20));
}