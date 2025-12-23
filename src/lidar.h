#pragma once
#include "config.h"

void tcaSelect(uint8_t channel);
bool tf02_read_full_esp32(MeasurementData &m);
int  tf02_init_i2c_mode();
