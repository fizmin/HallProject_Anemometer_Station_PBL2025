#include <Wire.h>
#include "lidar.h"
#include "config.h"

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

bool tf02_read_full_esp32(MeasurementData &m) {
  Wire.beginTransmission(TF02_I2C_ADDR);
  Wire.write(0x5A); Wire.write(0x05); Wire.write(0x00);
  Wire.write(0x01); Wire.write(0x60);
  if (Wire.endTransmission() != 0) { m.valid = false; return false; }

  uint8_t frame[9];
  if (Wire.requestFrom(TF02_I2C_ADDR, 9) != 9) { m.valid = false; return false; }
  for (int i = 0; i < 9; ++i) {
    if (!Wire.available()) { m.valid = false; return false; }
    frame[i] = Wire.read();
  }
  if (frame[0] != 0x59 || frame[1] != 0x59) { m.valid = false; return false; }

  uint8_t sum = 0;
  for (int i = 0; i < 8; ++i) sum += frame[i];
  if (sum != frame[8]) { m.valid = false; return false; }

  uint16_t dist_cm_raw = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);
  uint16_t strength    = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
  uint16_t temp_raw    = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);

  if (dist_cm_raw == 0xB1E0) { m.valid = false; return false; } // 45000

  m.dist_cm  = (float)dist_cm_raw;
  m.strength = strength;
  m.temp_c   = (float)temp_raw / 8.0f - 256.0f; // Temp/8-256 [web:4]
  m.valid    = true;
  return true;
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
