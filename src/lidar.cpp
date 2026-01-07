#include <Wire.h>
#include "lidar.h"
#include "config.h"

/**
 * Funkcja wybierająca kanał na multiplekserze I2C (TCA9548A).
 * Pozwala procesorowi komunikować się z konkretnym sensorem LIDAR.
 * @param channel Numer kanału (0-7)
 */
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  // Przesunięcie bitowe aktywuje odpowiednią linię wyjściową na multiplekserze
  Wire.write(1 << channel);
  Wire.endTransmission();
}

/**
 * Odczyt pełnej ramki danych z sensora TF02-i (LIDAR).
 * @param m Referencja do struktury, w której zapiszemy wyniki
 * @return true jeśli odczyt i suma kontrolna są poprawne
 */
bool tf02_read_full_esp32(MeasurementData &m) {
  // 1. Wysłanie komendy wyzwalającej pomiar (Trigger Command)
  Wire.beginTransmission(TF02_I2C_ADDR);
  Wire.write(0x5A); Wire.write(0x05); Wire.write(0x00);
  Wire.write(0x01); Wire.write(0x60);
  if (Wire.endTransmission() != 0) { m.valid = false; return false; }

  // 2. Odczyt 9-bajtowej ramki danych z sensora
  uint8_t frame[9];
  if (Wire.requestFrom(TF02_I2C_ADDR, 9) != 9) { m.valid = false; return false; }
  for (int i = 0; i < 9; ++i) {
    if (!Wire.available()) { m.valid = false; return false; }
    frame[i] = Wire.read();
  }

  // 3. Sprawdzenie nagłówka ramki (zawsze 0x59 0x59)
  if (frame[0] != 0x59 || frame[1] != 0x59) { m.valid = false; return false; }

  // 4. Weryfikacja sumy kontrolnej (Checksum)
  // Suma kontrolna to młodszy bajt sumy pierwszych 8 bajtów
  uint8_t sum = 0;
  for (int i = 0; i < 8; ++i) sum += frame[i];
  if (sum != frame[8]) { m.valid = false; return false; }

  // 5. Dekodowanie danych z ramki (Little Endian)
  uint16_t dist_cm_raw = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8); // Odległość w cm
  uint16_t strength    = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8); // Siła sygnału (wiarygodność)
  uint16_t temp_raw    = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8); // Temperatura procesora LIDARa

  // Sprawdzenie, czy sensor nie zwrócił wartości błędu (np. obiekt za daleko)
  if (dist_cm_raw == 0xB1E0) { m.valid = false; return false; } 

  // Przekonwertowanie danych na czytelne jednostki
  m.dist_cm  = (float)dist_cm_raw;
  m.strength = strength;
  // Przeliczenie temperatury zgodnie z dokumentacją TF02: T = temp_raw / 8 - 256
  m.temp_c   = (float)temp_raw / 8.0f - 256.0f;
  m.valid    = true;
  return true;
}

/**
 * Inicjalizacja sensora TF02 w trybie I2C.
 * Wysyła sekwencję komend konfiguracyjnych (częstotliwość, format danych).
 * @return 0 w przypadku sukcesu, ujemny kod błędu w przypadku niepowodzenia
 */
int tf02_init_i2c_mode() {
  // Każda sekwencja Wire.write to specyficzna komenda binarna producenta Benewake
  
  // Włączenie trybu wyzwalania pomiarów (Trigger mode)
  Wire.beginTransmission(TF02_I2C_ADDR);
  Wire.write(0xAA); Wire.write(0x55); Wire.write(0xF0); Wire.write(0x00);
  Wire.write(0x01); Wire.write(0x00); Wire.write(0x00); Wire.write(0x02);
  if (Wire.endTransmission() != 0) return -1;
  vTaskDelay(pdMS_TO_TICKS(100));

  // Ustawienie częstotliwości wyjściowej (Output rate)
  Wire.beginTransmission(TF02_I2C_ADDR);
  Wire.write(0x5A); Wire.write(0x05); Wire.write(0x0A); Wire.write(0x01); Wire.write(0x6A);
  if (Wire.endTransmission() != 0) return -2;
  vTaskDelay(pdMS_TO_TICKS(100));

  // Komenda aktywacji trybu I2C
  Wire.beginTransmission(TF02_I2C_ADDR);
  Wire.write(0x5A); Wire.write(0x04); Wire.write(0x11); Wire.write(0x6F);
  if (Wire.endTransmission() != 0) return -3;
  vTaskDelay(pdMS_TO_TICKS(100));

  // Zapisanie ustawień w pamięci nieulotnej sensora
  Wire.beginTransmission(TF02_I2C_ADDR);
  Wire.write(0xAA); Wire.write(0x55); Wire.write(0xF0); Wire.write(0x00);
  Wire.write(0x00); Wire.write(0x00); Wire.write(0x00); Wire.write(0x02);
  if (Wire.endTransmission() != 0) return -4;
  vTaskDelay(pdMS_TO_TICKS(100));

  // Finalizacja i restart sensora
  Wire.beginTransmission(TF02_I2C_ADDR);
  Wire.write(0xAA); Wire.write(0x55); Wire.write(0xF0); Wire.write(0x00);
  Wire.write(0xFF); Wire.write(0xFF); Wire.write(0xFF); Wire.write(0xFF);
  if (Wire.endTransmission() != 0) return -5;
  vTaskDelay(pdMS_TO_TICKS(200));

  return 0; // Sukces
}