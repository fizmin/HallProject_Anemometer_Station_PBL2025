#include <Arduino.h>
#include "display.h"
#include "config.h"
#include "oled/DEV_Config.h"
#include "oled/OLED_1in5_b.h"
#include "oled/GUI_Paint.h"
#include "oled/fonts.h"

// Odniesienie do zewnętrznego bufora obrazu zadeklarowanego w main.cpp
extern UBYTE Image_buf[];

/**
 * Zadanie RTOS obsługujące kontrolę wyświetlacza OLED.
 * Działa w pętli, sprawdzając timeout i odświeżając ekran.
 */
void oledControlTask(void* pv) {
  for (;;) {
    unsigned long now = millis();
    unsigned long dt = now - oledLastUpdateTime;

    // Automatyczne wyłączanie ekranu (wygaszacz) po upływie zdefiniowanego czasu
    if (oledEnabled && dt >= OLED_TIMEOUT_MS) {
      // Blokada muteksu na czas czyszczenia ekranu
      xSemaphoreTake(avgArrayMutex, pdMS_TO_TICKS(100));
      
      // Zamalowanie całego ekranu na czarno i wysłanie do sterownika
      Paint_DrawRectangle(0,0,128,128,BLACK,DOT_PIXEL_1X1,DRAW_FILL_FULL);
      OLED_1in5_B_Display(Image_buf);
      
      xSemaphoreGive(avgArrayMutex);
      oledEnabled = false; // Deaktywacja flagi wyświetlania
      Serial.println("[OLED] Timeout - screen off");
    }

    // Jeśli wyświetlacz jest aktywny, wywołaj funkcję rysującą interfejs
    if (oledEnabled) updateOLED();
    
    // Odświeżanie interfejsu co 500ms (2Hz), aby oszczędzać CPU
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

/**
 * Funkcja rysująca dane pomiarowe na ekranie OLED.
 */
void updateOLED() {
  // Próba przejęcia muteksu - jeśli dane są aktualizowane przez LidarTask, czekamy 100ms
  if (!xSemaphoreTake(avgArrayMutex, pdMS_TO_TICKS(100))) return;

  // Wyczyszczenie tła na biało (przygotowanie ramki)
  Paint_DrawRectangle(0,0,128,128,WHITE,DOT_PIXEL_1X1,DRAW_FILL_FULL);

  // --- OBSŁUGA OSI X (LIDARY 1 i 2) ---
  Paint_DrawString_EN(5, 3, "Os X", &Font16, BLACK, WHITE);

  float avg_x = 0;
  String instr_x = ""; // Instrukcja dla użytkownika (np. "Obróć")
  int valid_x = 0;     // Liczba sprawnych czujników na tej osi

  // Sprawdzenie poprawności odczytów z obu sensorów osi X
  if (avgArray[1] >= 0 && avgArray[2] >= 0) {
    avg_x = (avgArray[1] + avgArray[2]) / 2.0f + offsetX; // Średnia z dwóch sensorów + kalibracja
    float diff = avgArray[1] - avgArray[2];
    
    // Logika poziomowania: jeśli różnica jest duża, sugeruj obrót
    if (diff > 6) instr_x = "Obr CW";      // Zgodnie z ruchem wskazówek zegara
    else if (diff < -6) instr_x = "Obr CCW"; // Przeciwnie do ruchu wskazówek zegara
    else instr_x = "OK";                    // Oś wyrównana
    valid_x = 2;
  } else if (avgArray[1] >= 0) {
    avg_x = avgArray[1] + offsetX;
    instr_x = "L2 err"; // Awaria drugiego sensora
    valid_x = 1;
  } else if (avgArray[2] >= 0) {
    avg_x = avgArray[2] + offsetX;
    instr_x = "L1 err"; // Awaria pierwszego sensora
    valid_x = 1;
  } else {
    instr_x = "Both err"; // Oba sensory na osi X nie działają
  }

  // Wyświetlenie wyniku dla osi X
  char buf_x[16];
  if (valid_x > 0) sprintf(buf_x, "%.1f cm", avg_x);
  else sprintf(buf_x, "ERROR");
  
  Paint_DrawString_EN(5, 20, buf_x, &Font16, BLACK, WHITE);
  Paint_DrawString_EN(5, 36, instr_x.c_str(), &Font16, BLACK, WHITE);

  // Linia oddzielająca sekcje ekranu
  Paint_DrawLine(0, 54, 127, 54, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

  // --- OBSŁUGA OSI Y (LIDARY 3 i 4) ---
  Paint_DrawString_EN(5, 58, "Os Y", &Font16, BLACK, WHITE);

  float avg_y = 0;
  String instr_y = "";
  int valid_y = 0;

  if (avgArray[3] >= 0 && avgArray[4] >= 0) {
    avg_y = (avgArray[3] + avgArray[4]) / 2.0f + offsetY;
    float diff = avgArray[3] - avgArray[4];
    
    if (diff > 6) instr_y = "Obr CW";
    else if (diff < -6) instr_y = "Obr CCW";
    else instr_y = "OK";
    valid_y = 2;
  } else if (avgArray[3] >= 0) {
    avg_y = avgArray[3] + offsetY;
    instr_y = "L4 err";
    valid_y = 1;
  } else if (avgArray[4] >= 0) {
    avg_y = avgArray[4] + offsetY;
    instr_y = "L3 err";
    valid_y = 1;
  } else {
    instr_y = "Both err";
  }

  // Wyświetlenie wyniku dla osi Y
  char buf_y[16];
  if (valid_y > 0) sprintf(buf_y, "%.1f cm", avg_y);
  else sprintf(buf_y, "ERROR");
  
  Paint_DrawString_EN(5, 75, buf_y, &Font16, BLACK, WHITE);
  Paint_DrawString_EN(5, 91, instr_y.c_str(), &Font16, BLACK, WHITE);

  // Fizyczna aktualizacja matrycy OLED danymi z bufora
  OLED_1in5_B_Display(Image_buf);
  
  // Zwolnienie muteksu, aby LidarTask mógł zapisać nowe pomiary
  xSemaphoreGive(avgArrayMutex);
}