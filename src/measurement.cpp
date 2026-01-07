#include <Arduino.h>
#include <math.h>
#include <json.hpp>
#include "measurement.h"
#include "lidar.h"
#include "config.h"

using json = nlohmann::json;

// ================== FUNKCJE POMOCNICZE ==================

/**
 * Dodaje nowy pomiar do bufora kołowego wybranego lidara.
 * Pozwala to na podgląd historii pomiarów (np. komenda "raw").
 */
void pushToRawBuffer(uint8_t ch, const MeasurementData &m) {
  if (ch >= NUM_LIDARS) return;
  LidarBuffer &b = lidarBuffers[ch];
  b.buf[b.head] = m;
  b.head = (b.head + 1) % RAW_BUFFER_SIZE; // Przesunięcie głowicy (z zapętlaniem)
  if (b.count < RAW_BUFFER_SIZE) b.count++;
}

/**
 * Oblicza średnią arytmetyczną z tablicy pomiarów.
 */
float calcMean(const float *data, int n) {
  if (n <= 0) return NAN;
  float s = 0.0f;
  for (int i = 0; i < n; ++i) s += data[i];
  return s / (float)n;
}

/**
 * Oblicza odchylenie standardowe - wskaźnik stabilności pomiaru.
 * Duże odchylenie może oznaczać drgania lub przeszkodę na drodze wiązki.
 */
float calcStdDev(const float *data, int n, float mean) {
  if (n <= 1) return NAN;
  float s = 0.0f;
  for (int i = 0; i < n; ++i) {
    float d = data[i] - mean;
    s += d * d;
  }
  return sqrtf(s / (float)(n - 1));
}

// ================== TASK POMIAROWY (RTOS) ==================

void lidarMeasurementTask(void* pv) {
  for (;;) {
    // Jeśli nie ma żadnego żądania, uśpij zadanie na 50ms (oszczędność energii)
    if (!measureRequested && !testRequested && !rawRequested) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // ---------- TRYB RAW: wysłanie historii ostatnich pomiarów przez JSON ----------
    if (rawRequested) {
      rawRequested = false;

      xSemaphoreTake(rawBufMutex, portMAX_DELAY); // Blokada buforów na czas odczytu
      json root = json::array();

      for (uint8_t ch = 1; ch < NUM_LIDARS; ++ch) {
        json lid;
        lid["lidar"] = ch;
        lid["status"] = "OK";

        json arr = json::array();
        LidarBuffer &b = lidarBuffers[ch];

        // Odczyt danych z bufora kołowego od najstarszego do najnowszego
        for (uint8_t i = 0; i < b.count; ++i) {
          uint8_t idx = (b.head + RAW_BUFFER_SIZE - b.count + i) % RAW_BUFFER_SIZE;
          const MeasurementData &m = b.buf[idx];

          json one;
          one["dist_cm"]  = m.dist_cm;
          one["strength"] = m.strength;
          one["temp_c"]   = m.temp_c;
          one["valid"]    = m.valid;
          arr.push_back(one);
        }

        lid["samples"] = arr;
        root.push_back(lid);
      }

      xSemaphoreGive(rawBufMutex);
      Serial.println(root.dump().c_str()); // Wysłanie JSON na UART
      continue;
    }

    // ---------- TRYB TEST: szybki test łączności z każdym sensorem ----------
    if (testRequested) {
      testRequested = false;

      digitalWrite(RELAY_PIN, HIGH); // Włącz laser/wskaźnik na czas testu
      vTaskDelay(pdMS_TO_TICKS(50));

      json root = json::array();

      for (uint8_t ch = 1; ch < NUM_LIDARS; ++ch) {
        tcaSelect(ch); // Wybór kanału I2C
        MeasurementData m{};
        bool ok = tf02_read_full_esp32(m);

        json e;
        e["lidar"] = ch;

        if (ok) {
          e["status"]    = "OK";
          e["dist_cm"]   = m.dist_cm;
          e["strength"]  = m.strength;
          e["temp_c"]    = m.temp_c;
        } else {
          e["status"] = "error";
          e["reason"] = "strength_or_comm";
        }

        xSemaphoreTake(rawBufMutex, portMAX_DELAY);
        pushToRawBuffer(ch, m);
        xSemaphoreGive(rawBufMutex);

        root.push_back(e);
      }

      digitalWrite(RELAY_PIN, LOW); // Wyłącz laser
      vTaskDelay(pdMS_TO_TICKS(50));

      Serial.println(root.dump().c_str());
      continue;
    }

    // ---------- TRYB MEASURE: precyzyjna seria pomiarowa (Oś X i Y) ----------
    if (measureRequested) {
      measureRequested = false;

      digitalWrite(RELAY_PIN, HIGH); // Włącz laser wspomagający celowanie
      vTaskDelay(pdMS_TO_TICKS(50));

      float localAvg[NUM_LIDARS];
      float localStd[NUM_LIDARS];

      // Inicjalizacja wyników lokalnych
      for (uint8_t i = 0; i < NUM_LIDARS; ++i) {
        localAvg[i] = -1.0f;
        localStd[i] = NAN;
      }

      bool anyError = false;
      json errorInfo = json::array();

      // Pętla po wszystkich sensorach
      for (uint8_t ch = 1; ch < NUM_LIDARS; ++ch) {
        tcaSelect(ch);

        float vals[5]; // Bufor na serię 5 udanych pomiarów
        int   used     = 0;
        int   attempts = 0;

        // Próba zebrania 5 stabilnych odczytów (maksymalnie 10 prób)
        while (used < 5 && attempts < 10) {
          MeasurementData m{};
          bool ok = tf02_read_full_esp32(m);
          attempts++;

          xSemaphoreTake(rawBufMutex, portMAX_DELAY);
          pushToRawBuffer(ch, m);
          xSemaphoreGive(rawBufMutex);

          // Akceptujemy pomiar tylko jeśli sygnał jest silny (>60)
          if (ok && m.valid && m.strength > 60) {
            vals[used++] = m.dist_cm;
          }
          vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (used == 0) {
          anyError = true;
          json e;
          e["lidar"]  = ch;
          e["reason"] = "no_valid_measurements";
          errorInfo.push_back(e);
        } else {
          localAvg[ch] = calcMean(vals, used);
          localStd[ch] = calcStdDev(vals, used, localAvg[ch]);
        }
      }

      digitalWrite(RELAY_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(50));

      // Bezpieczna aktualizacja danych globalnych dla zadania OLED
      xSemaphoreTake(avgArrayMutex, portMAX_DELAY);
      for (uint8_t ch = 1; ch < NUM_LIDARS; ++ch) {
        avgArray[ch] = localAvg[ch];
        stdArray[ch] = localStd[ch];
      }
      xSemaphoreGive(avgArrayMutex);

      // ---------- Budowa raportu końcowego dla osi X i Y ----------
      json resp = json::array();

      // Przetwarzanie Pary 1–2 (Oś X) oraz Pary 3–4 (Oś Y)
      for (int axis_idx = 0; axis_idx < 2; axis_idx++) {
        uint8_t id1 = (axis_idx == 0) ? 1 : 3;
        uint8_t id2 = (axis_idx == 0) ? 2 : 4;
        float offset = (axis_idx == 0) ? offsetX : offsetY;

        json axis;
        axis["axis"] = (axis_idx == 0) ? "X" : "Y";
        axis["pair"] = (axis_idx == 0) ? "Lidar1_2" : "Lidar3_4";

        float l1 = localAvg[id1];
        float l2 = localAvg[id2];

        if (l1 < 0 || l2 < 0) {
          axis["status"]  = "error";
          axis["message"] = "invalid_lidar";
        } else {
          float meanPair  = (l1 + l2) / 2.0f + offset;
          // Obliczanie łącznego odchylenia (Pooled Standard Deviation)
          float pooledStd = sqrtf((localStd[id1] * localStd[id1] + localStd[id2] * localStd[id2]) / 2.0f);

          // Logika komendy obrotu (CW/CCW)
          float diff = l1 - l2;
          const char* cmd = "NONE";
          if (diff > 6.0f)      cmd = "CW";
          else if (diff < -6.0f) cmd = "CCW";

          axis["status"]     = "OK";
          axis["command"]    = cmd;
          axis["mean_cm"]    = meanPair;
          axis["std_cm"]     = pooledStd;
          axis["raw_l1_cm"]  = l1;
          axis["raw_l2_cm"]  = l2;
        }
        resp.push_back(axis);
      }

      if (anyError) {
        json err;
        err["axis"] = "errors";
        err["details"] = errorInfo;
        resp.push_back(err);
      }

      Serial.println(resp.dump().c_str());
    }
  }
}