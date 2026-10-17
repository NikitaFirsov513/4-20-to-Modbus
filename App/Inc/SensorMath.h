#pragma once
#include "ModbusRegisters.h"
/* Voltage register still uses the nominal ADC reference. Current is calibrated
 * directly from ADC counts, independently of this voltage conversion. */
#ifndef SENSOR_ADC_REFERENCE_UV
#define SENSOR_ADC_REFERENCE_UV 3300000UL
#endif
/* Measured 2026-10-17: ten samples at each endpoint, in 0.1 ADC counts.
 * 4 mA: mean 233.1; 20 mA: mean 3922.5. See calibration/2026-10-17.json.
 * These defaults belong to the measured board; recalibrate other boards. */
#ifndef SENSOR_CAL_RAW_4MA_X10
#define SENSOR_CAL_RAW_4MA_X10 2331L
#endif
#ifndef SENSOR_CAL_RAW_20MA_X10
#define SENSOR_CAL_RAW_20MA_X10 39225L
#endif
void Sensor_Convert(uint16_t raw, bool loop_open, ADC_data_t *data);

bool Sensor_SetCalibration(int32_t raw4_x10, int32_t raw20_x10);
void Sensor_GetCalibration(int32_t *raw4_x10, int32_t *raw20_x10);

typedef struct { uint8_t loop_count; bool loop_open, under, over; } SensorStatusFilter;
void Sensor_FilterStatus(SensorStatusFilter *filter, ADC_data_t *data);
