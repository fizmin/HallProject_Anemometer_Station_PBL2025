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
// volatile jest używane, ponieważ zmienne te są modyfikowane w różnych zadaniach (taskach)
volatile bool measureRequested = false; // Flaga żądania pomiaru
volatile bool testRequested    = false; // Flaga żądania testu systemu
volatile bool rawRequested     = false; // Flaga żądania surowych danych

volatile unsigned long oledLastUpdateTime = 0; // Czas ostatniej aktualizacji wyświetlacza
volatile bool oledEnabled = true;               // Czy wyświetlacz ma być aktywny
volatile bool relayState  = false;              // Aktualny stan przekaźnika (np. lasera)

// Muteksy do bezpiecznej wymiany danych między rdzeniami ESP32
SemaphoreHandle_t avgArrayMutex = nullptr; // Muteks dla tablicy uśrednionych wyników
SemaphoreHandle_t rawBufMutex   = nullptr; // Muteks dla buforów surowych danych

// Tablice przechowujące wyniki pomiarów dla każdego z LIDARów
float avgArray[NUM_LIDARS];      // Średnia odległość
float stdArray[NUM_LIDARS];      // Odchylenie standardowe (stabilność pomiaru)
LidarBuffer lidarBuffers[NUM_LIDARS]; // Bufor kołowy dla każdego sensora

// Parametry kalibracyjne (offsety geometryczne)
float offsetX = 0.2f;
float offsetY = 0.2f;

// Bufor obrazu dla wyświetlacza OLED 1.5"
UBYTE Image_buf[OLED_1in5_B_WIDTH * OLED_1in5_B_HEIGHT / 8];

void setup() {
  Wire.begin();           // Inicjalizacja magistrali I2C
  Serial.begin(115200);   // Inicjalizacja komunikacji szeregowej
  delay(500);

  // Konfiguracja pinów sterujących
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLDOWN); // Przycisk z rezystorem do masy
  relayState = false;
  digitalWrite(RELAY_PIN, LOW);

  // Inicjalizacja tablic pomiarowych wartościami domyślnymi
  for (uint8_t i=0; i<NUM_LIDARS; ++i){
    avgArray[i] = -1.0f;
    stdArray[i] = NAN;
    lidarBuffers[i].head = 0;
    lidarBuffers[i].count = 0;
  }

  // Tworzenie semaforów binarnych (muteksów)
  avgArrayMutex = xSemaphoreCreateMutex();
  rawBufMutex   = xSemaphoreCreateMutex();
  // Jeśli nie udało się utworzyć muteksów, zatrzymaj program (bezpiecznik)
  if (!avgArrayMutex || !rawBufMutex) { while(1) delay(1000); }

  // Inicjalizacja fizyczna ekranu OLED i wyczyszczenie bufora
  OLED_1in5_B_Init();
  Paint_NewImage(Image_buf, OLED_1in5_B_WIDTH, OLED_1in5_B_HEIGHT, ROTATE_0, WHITE);
  OLED_1in5_B_Display(Image_buf);
  oledLastUpdateTime = millis();

  // Inicjalizacja sensorów TF02-i2c przez multiplekser TCA9548A
  for (uint8_t ch=1; ch<5; ++ch) {
    tcaSelect(ch); // Przełączenie kanału na multiplekserze
    int r = tf02_init_i2c_mode(); // Próba nawiązania łączności z sensorem
    Serial.print("CH"); Serial.print(ch);
    Serial.println(r==0?": OK":": FAIL");
  }

  // Tworzenie zadań FreeRTOS i przypisanie ich do konkretnych rdzeni ESP32
  // Rdzeń 1: Pomiary LIDAR (wysoki priorytet)
  xTaskCreatePinnedToCore(lidarMeasurementTask, "LidarTask", 8192, nullptr, 2, nullptr, 1);
  // Rdzeń 0: Obsługa interfejsu użytkownika i OLED (niższy priorytet)
  xTaskCreatePinnedToCore(oledControlTask, "OledTask", 4096, nullptr, 1, nullptr, 0);

  Serial.println("Ready");
}

void loop() {
  // --- 1. Obsługa poleceń JSON przychodzących przez UART ---
  if (Serial.available() > 0) {
    static String buffer = "";
    char c = (char)Serial.read();
    buffer += c;

    // Sprawdzenie, czy otrzymano kompletny obiekt JSON (domknięcie nawiasu)
    if (buffer.indexOf('}') != -1) {
      try {
        json cmd = json::parse(buffer.c_str());
        buffer = ""; // Czyszczenie bufora po parsowaniu

        // Obsługa poszczególnych komend
        if (cmd.contains("measure")) {
          measureRequested = true;
          oledLastUpdateTime = millis();
          oledEnabled = true;
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
        // Obsługa błędów w formacie JSON
        Serial.println("{\"status\":\"error\",\"reason\":\"json_parse\"}");
        buffer = "";
      }
    }
  }

  // --- 2. Obsługa fizycznego przycisku (Logika Toggle) ---
  static bool lastButtonState = LOW;
  bool currentButtonState = digitalRead(BUTTON_PIN);

  // Detekcja zbocza narastającego (naciśnięcia przycisku)
  if (currentButtonState == HIGH && lastButtonState == LOW) {
    relayState = !relayState; // Zmiana stanu logicznego
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW); // Przełączenie pinu przekaźnika
    Serial.print("[BUTTON] Relay (laser) ");
    Serial.println(relayState ? "ON" : "OFF");
  }
  lastButtonState = currentButtonState;

  // Krótkie opóźnienie, aby oddać czas procesora innym procesom w pętli loop
  vTaskDelay(pdMS_TO_TICKS(20));
}