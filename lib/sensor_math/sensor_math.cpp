#include <Arduino.h>
#include "sensor_math.h"

// ADC constansts
constexpr float VREF = 3.3f;
constexpr float ADC_MAX = 4095.0f; // 12-bit analog resolution

// Pressure sensor constants
// Voltage divider resistors: Rtop = 5.6 kΩ, Rbot = 10 kΩ
constexpr float DIVIDER_RATIO = 10.0f / (5.6f + 10.0f);
// Sensor characteristic: 0.5V @ 0 bar, 4.5V @ 10 bar
constexpr float OFFSET_VOLTAGE = 0.5f;
constexpr float FULL_SCALE_VOLTAGE = 4.5f;
constexpr float SENSITIVITY_BAR_PER_VOLT = 10.0f / (FULL_SCALE_VOLTAGE - OFFSET_VOLTAGE);
constexpr float PSI_PER_BAR = 14.5038f;

// Temperature sensor constants
// Divider top resistor: 6.8 kΩ
constexpr float R_TOP = 6800.0f;
constexpr float SH_A = 0.0012885499f;
constexpr float SH_B = 2.6171841707e-04f;
constexpr float SH_C = 1.6110455605e-07f;

/**
 * @brief Convert raw ADC reading to voltage.
 *
 * Converts a 12-bit ADC value (0–4095) to its corresponding voltage based on VREF.
 *
 * @param raw ADC count (0–4095).
 * @return Voltage in volts (0.0–3.3 V).
 */
float adcToVoltage(int raw) {
	return raw * VREF / ADC_MAX;
}

/**
 * @brief Convert ADC voltage to oil pressure in psi.
 *
 * Applies the inverse of the voltage divider and sensor linear mapping
 * to compute the pressure in bar, then converts to psi.
 *
 * @param vAdc Voltage from ADC input (after divider), in volts.
 * @return Pressure in psi. Returns NaN if vAdc is out of range.
 */
float computePressurePsi(float vAdc) {
    if (vAdc <= 0.0f || vAdc >= VREF) return NAN;

    // Compute actual sensor voltage before divider
    float vSensor = vAdc / DIVIDER_RATIO;
    // Calculate pressure in bar
    float pressureBar = (vSensor - OFFSET_VOLTAGE) * SENSITIVITY_BAR_PER_VOLT;
    // Convert to psi
    return pressureBar * PSI_PER_BAR;
}

/**
 * @brief Convert ADC voltage to temperature in degrees Celsius.
 *
 * Computes thermistor resistance from ADC voltage and applies
 * the Steinhart–Hart equation to estimate temperature.
 *
 * @param vAdc Voltage from ADC input (after divider), in volts.
 * @return Temperature in °C. Returns NaN if vAdc is out of range.
 */
float computeTemperatureC(float vAdc) {
    if (vAdc <= 0.0f || vAdc >= VREF) return NAN;

    // Compute sensor resistance from divider equation: Vout = VREF * (R_sensor/(R_top + R_sensor))
    float rSensor = R_TOP * (vAdc / (VREF - vAdc));
    // Compute temperature via Steinhart–Hart equation
    float lnR = log(rSensor);
    float invT = SH_A + SH_B * lnR + SH_C * lnR * lnR * lnR;
    float tempK = 1.0f / invT;
    return tempK - 273.15f;
}
