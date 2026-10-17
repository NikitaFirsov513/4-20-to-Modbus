#include "SensorMath.h"

_Static_assert(SENSOR_CAL_RAW_4MA_X10 >= 0L, "Invalid 4 mA calibration point");
_Static_assert(SENSOR_CAL_RAW_20MA_X10 <= 40950L, "Invalid 20 mA calibration point");
_Static_assert(SENSOR_CAL_RAW_20MA_X10 > SENSOR_CAL_RAW_4MA_X10,
               "Calibration points must increase");

static int32_t calibration_low = SENSOR_CAL_RAW_4MA_X10;
static int32_t calibration_high = SENSOR_CAL_RAW_20MA_X10;
bool Sensor_SetCalibration(int32_t low, int32_t high)
{
    if (low < 0 || high > 40950 || high <= low) return false;
    calibration_low = low; calibration_high = high; return true;
}
void Sensor_GetCalibration(int32_t *low, int32_t *high) { *low = calibration_low; *high = calibration_high; }

void Sensor_Convert(uint16_t raw, bool loop_open, ADC_data_t *data)
{
    uint32_t uv = ((uint64_t)raw * SENSOR_ADC_REFERENCE_UV + 2047U) / 4095U;
    const int64_t span = calibration_high - calibration_low;
    /* I[uA] = 4000 + (raw*10 - raw4_x10)*16000/span.
     * Round once to the nearest microamp; extrapolate beyond the endpoints
     * so that under/overrange diagnostics remain meaningful. */
    int64_t numerator = 4000LL * span +
        ((int64_t)raw * 10LL - calibration_low) * 16000LL;
    int64_t calibrated = numerator <= 0LL ? 0LL : (numerator + span / 2LL) / span;
    uint32_t ua = calibrated > 65535LL ? 65535U : (uint32_t)calibrated;
    data->adc_raw = raw;
    data->adc_mv = (uint16_t)((uv + 500U) / 1000U);
    data->current_uA = (uint16_t)(ua > 65535U ? 65535U : ua);
    data->status = loop_open ? ADC_STATUS_LOOP_OPEN : 0U;
    if (ua < 3800U || loop_open) data->status |= ADC_STATUS_UNDERRANGE;
    if (ua > 20500U) data->status |= ADC_STATUS_OVERRANGE;
    if (raw >= 4090U) data->status |= ADC_STATUS_SATURATED;
}

void Sensor_FilterStatus(SensorStatusFilter *f, ADC_data_t *data)
{
    bool fault = (data->status & ADC_STATUS_LOOP_OPEN) != 0U;
    if (fault) { if (f->loop_count < 3U) ++f->loop_count; }
    else if (f->loop_count) --f->loop_count;
    if (f->loop_count == 3U) f->loop_open = true;
    if (!f->loop_count) f->loop_open = false;
    if (data->current_uA < 3800U) f->under = true;
    else if (data->current_uA > 3900U) f->under = false;
    if (data->current_uA > 20500U) f->over = true;
    else if (data->current_uA < 20400U) f->over = false;
    data->status &= ~(ADC_STATUS_LOOP_OPEN | ADC_STATUS_UNDERRANGE | ADC_STATUS_OVERRANGE);
    if (f->loop_open) data->status |= ADC_STATUS_LOOP_OPEN;
    if (f->under || f->loop_open) data->status |= ADC_STATUS_UNDERRANGE;
    if (f->over) data->status |= ADC_STATUS_OVERRANGE;
}
