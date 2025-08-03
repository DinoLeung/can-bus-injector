#include <Arduino.h>
#include "globals.h"
#include "sensor.h"
#include "sensor_math.h"
#include "sensor_tasks.h"

// PST-F1 signal pins
#define PRESSURE_PIN GPIO_NUM_35
#define TEMPERATURE_PIN GPIO_NUM_36

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
 * @brief FreeRTOS task that samples the analog voltage from the oil pressure sensor,
 *        computes actual sensor voltage, and calculates oil pressure in bar and psi.
 *
 * Reads ADC from OIL_PRESSURE pin, converts to voltage, undoes voltage divider,
 * converts to pressure (bar and psi), and update `g_oilPressurePsi10` every 25 ms (40 Hz).
 */
void pressureSamplingTask(void* pvParameters) {
	(void)pvParameters;
	TickType_t lastWake = xTaskGetTickCount();
	while (true) {
		int raw = analogRead(PRESSURE_PIN);
		float vAdc = adcToVoltage(raw);
		float pressurePsi = computePressurePsi(vAdc);

		if (isnan(pressurePsi)) {
			g_oilPressurePsi10 = INT16_MAX;
		} else {
			g_oilPressurePsi10 = static_cast<uint16_t>(pressurePsi * 10.0f);
		}

		vTaskDelayUntil(&lastWake, sensorSampleInterval);
	}
}

/**
 * @brief FreeRTOS task that samples the analog voltage from the oil temperature NTC sensor,
 *        computes sensor resistance, and converts it to temperature in °C.
 *
 * Reads ADC from OIL_TEMP pin, converts to voltage, computes sensor resistance via divider equation,
 * converts resistance to temperature using the Steinhart–Hart equation, and update `g_oilTempC10` every 25 ms (40 Hz).
 *
 */
void temperatureSamplingTask(void* pvParameters) {
	(void)pvParameters;
	TickType_t lastWake = xTaskGetTickCount();
	while (true) {
		int raw = analogRead(TEMPERATURE_PIN);
		float vAdc = adcToVoltage(raw);
		float tempC = computeTemperatureC(vAdc);

		if (isnan(tempC)) {
			g_oilTempC10 = INT16_MAX;
		} else {
			g_oilTempC10 = static_cast<int16_t>(tempC * 10.0f);
		}

		vTaskDelayUntil(&lastWake, sensorSampleInterval);
	}
}
