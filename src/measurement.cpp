#include <Arduino.h>
#include <math.h>
#include <json.hpp>
#include "measurement.h"
#include "lidar.h"
#include "config.h"

using json = nlohmann::json;

// ================== FUNKCJE POMOCNICZE ==================

void pushToRawBuffer(uint8_t ch, const MeasurementData &m) {
  if (ch >= NUM_LIDARS) return;
  LidarBuffer &b = lidarBuffers[ch];
  b.buf[b.head] = m;
  b.head = (b.head + 1) % RAW_BUFFER_SIZE;
  if (b.count < RAW_BUFFER_SIZE) b.count++;
}

float calcMean(const float *data, int n) {
  if (n <= 0) return NAN;
  float s = 0.0f;
  for (int i = 0; i < n; ++i) s += data[i];
  return s / (float)n;
}

float calcStdDev(const float *data, int n, float mean) {
  if (n <= 1) return NAN;
  float s = 0.0f;
  for (int i = 0; i < n; ++i) {
    float d = data[i] - mean;
    s += d * d;
  }
  return sqrtf(s / (float)(n - 1));
}

// ================== TASK POMIAROWA ==================

void lidarMeasurementTask(void* pv) {
  for (;;) {
    // nic do roboty
    if (!measureRequested && !testRequested && !rawRequested) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // ---------- RAW: wysłanie 20 ostatnich pomiarów ----------
    if (rawRequested) {
      rawRequested = false;

      xSemaphoreTake(rawBufMutex, portMAX_DELAY);
      json root = json::array();

      for (uint8_t ch = 1; ch < NUM_LIDARS; ++ch) {
        json lid;
        lid["lidar"] = ch;
        lid["status"] = "OK";

        json arr = json::array();
        LidarBuffer &b = lidarBuffers[ch];

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
      Serial.println(root.dump().c_str());
      continue;
    }

    // ---------- TEST: pojedynczy pomiar z każdego lidara ----------
    if (testRequested) {
      testRequested = false;

      digitalWrite(RELAY_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(50));

      json root = json::array();

      for (uint8_t ch = 1; ch < NUM_LIDARS; ++ch) {
        tcaSelect(ch);
        MeasurementData m{};
        bool ok = tf02_read_full_esp32(m);

        json e;
        e["lidar"] = ch;

        if (ok) {
          e["status"]    = "OK";
          e["dist_cm"]   = m.dist_cm;
          e["std_cm"]    = nullptr;  // brak serii
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

      digitalWrite(RELAY_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(50));

      Serial.println(root.dump().c_str());
      continue;
    }

    // ---------- MEASURE: seria 5 pomiarów, Strength>60 ----------
    if (measureRequested) {
      measureRequested = false;

      digitalWrite(RELAY_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(50));

      float localAvg[NUM_LIDARS];
      float localStd[NUM_LIDARS];

      for (uint8_t i = 0; i < NUM_LIDARS; ++i) {
        localAvg[i] = -1.0f;
        localStd[i] = NAN;
      }

      bool anyError = false;
      json errorInfo = json::array();

      for (uint8_t ch = 1; ch < NUM_LIDARS; ++ch) {
        tcaSelect(ch);

        float vals[5];
        int   used     = 0;
        int   attempts = 0;

        while (used < 5 && attempts < 10) {
          MeasurementData m{};
          bool ok = tf02_read_full_esp32(m);
          attempts++;

          xSemaphoreTake(rawBufMutex, portMAX_DELAY);
          pushToRawBuffer(ch, m);
          xSemaphoreGive(rawBufMutex);

          if (ok && m.valid && m.strength > 60) {
            vals[used++] = m.dist_cm;
          }

          vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (used == 0) {
          localAvg[ch] = -1.0f;
          localStd[ch] = NAN;
          anyError = true;

          json e;
          e["lidar"]  = ch;
          e["reason"] = "no_valid_measurements";
          errorInfo.push_back(e);
        } else {
          float m = calcMean(vals, used);
          float s = calcStdDev(vals, used, m);
          localAvg[ch] = m;
          localStd[ch] = s;
        }
      }

      digitalWrite(RELAY_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(50));

      // zapis średnich/odchyleń do tablic globalnych (OLED i przyszłe użycie)
      xSemaphoreTake(avgArrayMutex, portMAX_DELAY);
      for (uint8_t ch = 1; ch < NUM_LIDARS; ++ch) {
        avgArray[ch] = localAvg[ch];
        stdArray[ch] = localStd[ch];
      }
      xSemaphoreGive(avgArrayMutex);

      // ---------- budowa odpowiedzi JSON dla osi X i Y ----------
      json resp = json::array();

      // Para 1–2 -> oś X
      {
        json axis;
        axis["axis"] = "X";
        axis["pair"] = "Lidar1_2";

        float l1 = localAvg[1];
        float l2 = localAvg[2];
        float s1 = localStd[1];
        float s2 = localStd[2];

        if (l1 < 0 || l2 < 0) {
          axis["status"]  = "error";
          axis["message"] = "invalid_lidar";
        } else {
          float meanPair  = (l1 + l2) / 2.0f;
          float pooledStd = sqrtf((s1 * s1 + s2 * s2) / 2.0f);

          // dodanie offsetu X
          meanPair += offsetX;

          float diff = l1 - l2;
          String cmd;
          if (diff > 6.0f)      cmd = "CW";
          else if (diff < -6.0f) cmd = "CCW";
          else                   cmd = "NONE";

          axis["status"]     = "OK";
          axis["command"]    = String(cmd).c_str();
          axis["mean_cm"]    = meanPair;
          axis["std_cm"]     = pooledStd;
          axis["raw_l1_cm"]  = l1;
          axis["raw_l2_cm"]  = l2;
        }

        resp.push_back(axis);
      }

      // Para 3–4 -> oś Y
      {
        json axis;
        axis["axis"] = "Y";
        axis["pair"] = "Lidar3_4";

        float l3 = localAvg[3];
        float l4 = localAvg[4];
        float s3 = localStd[3];
        float s4 = localStd[4];

        if (l3 < 0 || l4 < 0) {
          axis["status"]  = "error";
          axis["message"] = "invalid_lidar";
        } else {
          float meanPair  = (l3 + l4) / 2.0f;
          float pooledStd = sqrtf((s3 * s3 + s4 * s4) / 2.0f);

          // dodanie offsetu Y
          meanPair += offsetY;

          float diff = l3 - l4;
          String cmd;
          if (diff > 6.0f)      cmd = "CW";
          else if (diff < -6.0f) cmd = "CCW";
          else                   cmd = "NONE";

          axis["status"]     = "OK";
          axis["command"]    = String(cmd).c_str();
          axis["mean_cm"]    = meanPair;
          axis["std_cm"]     = pooledStd;
          axis["raw_l3_cm"]  = l3;
          axis["raw_l4_cm"]  = l4;
        }

        resp.push_back(axis);
      }

      // jeśli były błędy pojedynczych lidarów – dołącz je
      if (anyError) {
        json err;
        err["axis"]    = "errors";
        err["details"] = errorInfo;
        resp.push_back(err);
      }

      Serial.println(resp.dump().c_str());
    }
  }
}
