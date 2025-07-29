#include "analog_utils.h"
constexpr float VREF = 3.3f;
constexpr float ADC_MAX = 4095.0f;

/**
 * @brief Convert raw ADC reading to voltage (0–3.3V).
 * @param raw ADC count (0–4095).
 * @return Voltage corresponding to raw ADC value.
 */
float rawToVoltage(int raw) {
	return raw * VREF / ADC_MAX;
}