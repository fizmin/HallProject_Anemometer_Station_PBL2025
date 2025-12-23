#include <Arduino.h>
#include "display.h"
#include "config.h"
#include "oled/DEV_Config.h"
#include "oled/OLED_1in5_b.h"
#include "oled/GUI_Paint.h"
#include "oled/fonts.h"

extern UBYTE Image_buf[];

void oledControlTask(void* pv) {
  for (;;) {
    unsigned long now = millis();
    unsigned long dt = now - oledLastUpdateTime;

    if (oledEnabled && dt >= OLED_TIMEOUT_MS) {
      xSemaphoreTake(avgArrayMutex, pdMS_TO_TICKS(100));
      Paint_DrawRectangle(0,0,128,128,BLACK,DOT_PIXEL_1X1,DRAW_FILL_FULL);
      OLED_1in5_B_Display(Image_buf);
      xSemaphoreGive(avgArrayMutex);
      oledEnabled = false;
      Serial.println("[OLED] Timeout - screen off");
    }

    if (oledEnabled) updateOLED();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void updateOLED() {
  if (!xSemaphoreTake(avgArrayMutex, pdMS_TO_TICKS(100))) return;
  Paint_DrawRectangle(0,0,128,128,WHITE,DOT_PIXEL_1X1,DRAW_FILL_FULL);

  // Oś X (1+2)
  Paint_DrawString_EN(5, 3, "Os X", &Font16, BLACK, WHITE);

  float avg_x = 0;
  String instr_x = "";
  int valid_x = 0;

  if (avgArray[1] >= 0 && avgArray[2] >= 0) {
    avg_x = (avgArray[1] + avgArray[2]) / 2.0f + offsetX;
    float diff = avgArray[1] - avgArray[2];
    if (diff > 6) instr_x = "Obr CW";
    else if (diff < -6) instr_x = "Obr CCW";
    else instr_x = "OK";
    valid_x = 2;
  } else if (avgArray[1] >= 0) {
    avg_x = avgArray[1] + offsetX;
    instr_x = "L2 err";
    valid_x = 1;
  } else if (avgArray[2] >= 0) {
    avg_x = avgArray[2] + offsetX;
    instr_x = "L1 err";
    valid_x = 1;
  } else {
    instr_x = "Both err";
  }

  char buf_x[16];
  if (valid_x > 0) sprintf(buf_x, "%.1f cm", avg_x);
  else sprintf(buf_x, "ERROR");
  Paint_DrawString_EN(5, 20, buf_x, &Font16, BLACK, WHITE);
  Paint_DrawString_EN(5, 36, instr_x.c_str(), &Font16, BLACK, WHITE);

  Paint_DrawLine(0, 54, 127, 54, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

  // Oś Y (3+4)
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

  char buf_y[16];
  if (valid_y > 0) sprintf(buf_y, "%.1f cm", avg_y);
  else sprintf(buf_y, "ERROR");
  Paint_DrawString_EN(5, 75, buf_y, &Font16, BLACK, WHITE);
  Paint_DrawString_EN(5, 91, instr_y.c_str(), &Font16, BLACK, WHITE);

  OLED_1in5_B_Display(Image_buf);
  xSemaphoreGive(avgArrayMutex);
}
