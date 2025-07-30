#include "sensor.h"
#include "sensor_tasks.h"
#include "globals.h"
#include <Arduino.h>

// PST-F1 signal pins
#define PRESSURE_PIN GPIO_NUM_35
#define TEMPERATURE_PIN GPIO_NUM_36

constexpr float VREF = 3.3f;
constexpr float ADC_MAX = 4095.0f; // 12-bit analog resolution

// 40Hz
constexpr TickType_t sensorSampleInterval = pdMS_TO_TICKS(25);

void pressureSamplingTask(void*);
void temperatureSamplingTask(void*);

void startSensorTasks() {
	// Configure ADC resolution and attenuation for analog sensors
	analogReadResolution(12);
	analogSetPinAttenuation(PRESSURE_PIN, ADC_11db);
	analogSetPinAttenuation(TEMPERATURE_PIN, ADC_11db);

	xTaskCreate(pressureSamplingTask, "Oil_Pres", 2048, NULL, 1, NULL);
	xTaskCreate(temperatureSamplingTask, "Oil_Temp", 2048, NULL, 1, NULL);
}

/**
 * @brief Convert raw ADC reading to voltage (0–3.3V).
 * @param raw ADC count (0–4095).
 * @return Voltage corresponding to raw ADC value.
 */
float rawToVoltage(int raw) {
	return raw * VREF / ADC_MAX;
}

/**
 * @brief FreeRTOS task that samples the analog voltage from the oil pressure sensor,
 *        computes actual sensor voltage, and calculates oil pressure in bar and psi.
 *
 * Reads ADC from OIL_PRESSURE pin, converts to voltage, undoes voltage divider,
 * converts to pressure (bar and psi), and update `g_oilPressurePsi10` every 25 ms (40 Hz).
 */
void pressureSamplingTask(void* pvParameters) {
	(void)pvParameters;
	// Voltage divider resistors: Rtop = 5.6 kΩ, Rbot = 10 kΩ
	constexpr float DIVIDER_RATIO = 10.0f / (5.6f + 10.0f);
	// Sensor characteristic: 0.5V @ 0 bar, 4.5V @ 10 bar
	constexpr float OFFSET_VOLTAGE = 0.5f;
	constexpr float FULL_SCALE_VOLTAGE = 4.5f;
	constexpr float SENSITIVITY_BAR_PER_VOLT = 10.0f / (FULL_SCALE_VOLTAGE - OFFSET_VOLTAGE); // 2.5 bar/V
	constexpr float BAR_TO_PSI = 14.5038f;

	TickType_t lastWake = xTaskGetTickCount();
	while (true) {
		int raw = analogRead(PRESSURE_PIN);
		float vAdc = rawToVoltage(raw);
		// Compute actual sensor voltage before divider
		float vSensor = vAdc / DIVIDER_RATIO;
		// Calculate pressure in bar
		float pressureBar = (vSensor - OFFSET_VOLTAGE) * SENSITIVITY_BAR_PER_VOLT;
		// Convert to psi
		float pressurePsi = pressureBar * BAR_TO_PSI;

		Serial.printf(
			"Oil Pressure raw: %d, Vadc: %.3f V, Vsensor: %.3f V, Pressure: %.2f bar (%.2f psi)\n",
			raw, vAdc, vSensor, pressureBar, pressurePsi
		);

		g_oilPressurePsi10 = static_cast<uint16_t>(pressurePsi * 10.0f);

		vTaskDelayUntil(&lastWake, sensorSampleInterval);
	}
}

/**
 * @brief FreeRTOS task that samples the analog voltage from the oil temperature NTC sensor,
 *        computes sensor resistance, and converts it to temperature in °C.
 *
 * Reads ADC from OIL_TEMP pin, converts to voltage, computes sensor resistance via divider equation,
 * converts resistance to temperature using the Beta parameter method, and update `g_oilTempC10` every 25 ms (40 Hz).
 *
 */
void temperatureSamplingTask(void* pvParameters) {
	(void)pvParameters;
	// Divider top resistor: 30 kΩ
	constexpr float R_TOP = 30000.0f;
	// Steinhart–Hart coefficients (from 21-point least-squares fit, see temp_sensor_steinhart_hart.py)
	constexpr float SH_A = 0.0012885499f;
	constexpr float SH_B = 2.6171841707e-04f;
	constexpr float SH_C = 1.6110455605e-07f;

	TickType_t lastWake = xTaskGetTickCount();
	while (true) {
		int raw = analogRead(TEMPERATURE_PIN);
		float vAdc = rawToVoltage(raw);
		// Compute sensor resistance from divider equation: Vout = VREF * (R_sensor/(R_top + R_sensor))
		float rSensor = R_TOP * (vAdc / (VREF - vAdc));
		// Compute temperature via Steinhart–Hart equation
		float lnR = log(rSensor);
		float invT = SH_A
					+ SH_B * lnR
					+ SH_C * lnR * lnR * lnR;
		float tempK = 1.0f / invT;
		float tempC = tempK - 273.15f;

		Serial.printf(
			"Oil Temp raw: %d, Vadc: %.3f V, Rsen: %.1f Ω, Temp: %.2f °C\n",
			raw, vAdc, rSensor, tempC
		);

		g_oilTempC10 = static_cast<int16_t>(tempC * 10.0f);

		vTaskDelayUntil(&lastWake, sensorSampleInterval);
	}
}
