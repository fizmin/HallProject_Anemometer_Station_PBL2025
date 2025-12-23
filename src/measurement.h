#pragma once
#include "config.h"

void lidarMeasurementTask(void* pv);
void pushToRawBuffer(uint8_t ch, const MeasurementData &m);
float calcMean(const float *data, int n);
float calcStdDev(const float *data, int n, float mean);
