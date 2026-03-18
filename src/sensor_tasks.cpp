#include "sensor_readings.h"
#include "sensor_math.h"
#include "can_bus.h"

// PST-F1 signal pins
#define PRESSURE_PIN GPIO_NUM_35
#define TEMPERATURE_PIN GPIO_NUM_36

// Custom CAN ID
#define CAN_ID 0x7e0

// 40Hz
constexpr TickType_t sensorSampleInterval = pdMS_TO_TICKS(25);
// 20Hz
constexpr TickType_t sensorCanMsgInterval = pdMS_TO_TICKS(50);

void pressureSamplingTask(void*);
void temperatureSamplingTask(void*);
static void sensorCanWriterTask(void*);

void startSensorTasks() {
	// Configure ADC resolution and attenuation for analog sensors
	analogReadResolution(12);
	analogSetPinAttenuation(PRESSURE_PIN, ADC_11db);
	analogSetPinAttenuation(TEMPERATURE_PIN, ADC_11db);

	xTaskCreate(pressureSamplingTask, "Oil_Pres", 2048, NULL, 1, NULL);
	xTaskCreate(temperatureSamplingTask, "Oil_Temp", 2048, NULL, 1, NULL);
	xTaskCreate(sensorCanWriterTask, "Sensor_CAN_Writer", 2048, NULL, 1, NULL);
}

/**
 * @brief FreeRTOS task that samples and processes oil pressure sensor readings.
 *
 * This task runs at 40 Hz. It reads the analog voltage from the oil pressure sensor,
 * converts it to voltage using adcToVoltage(), then computes pressure in psi via computePressurePsi().
 * The computed value is scaled to 0.1 psi resolution and stored in g_oilPressurePsi10.
 * If the ADC voltage is invalid, g_oilPressurePsi10 is set to INT16_MAX.
 *
 * @param pvParameters Unused.
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
 * @brief FreeRTOS task that samples and processes oil temperature sensor readings.
 *
 * This task runs at 40 Hz. It reads the analog voltage from the NTC temperature sensor,
 * converts it to voltage using adcToVoltage(), then computes temperature in Celsius
 * via computeTemperatureC(). The computed value is scaled to 0.1 °C resolution and stored in g_oilTempC10.
 * If the ADC voltage is invalid, g_oilTempC10 is set to INT16_MAX.
 *
 * @param pvParameters Unused.
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

/**
 * @brief FreeRTOS task that periodically sends combined sensor values to CAN2.
 *
 * This task constructs a 4-byte CAN frame every 50 ms (20 Hz) that includes:
 * - Bytes 0–1: Oil pressure in deci-psi (uint16_t)
 * - Bytes 2–3: Oil temperature in deci-degrees Celsius (int16_t, two's complement)
 *
 * The values are read from shared volatile variables `g_oilPressurePsi10` and `g_oilTempC10`,
 * and sent as a standard 11-bit CAN frame using the identifier defined by `CAN_ID`.
 *
 * @param pvParameters Unused parameter for FreeRTOS compatibility.
 */
void sensorCanWriterTask(void* pvParameters) {
	(void)pvParameters;
	TickType_t lastWake = xTaskGetTickCount();
	while (true) {
		twai_message_t msg{};
		// Pack signed temperature (deci-°C) as two's‑complement uint16_t
		uint16_t rawTemp = static_cast<uint16_t>(g_oilTempC10);
		msg.identifier = CAN_ID;
		msg.extd = 0;
		msg.data_length_code = 4;
		msg.data[0] = uint8_t(g_oilPressurePsi10 >> 8);
		msg.data[1] = uint8_t(g_oilPressurePsi10 & 0xFF);
		msg.data[2] = uint8_t(rawTemp >> 8);
		msg.data[3] = uint8_t(rawTemp & 0xFF);
		writeCan2(msg);
		vTaskDelayUntil(&lastWake, sensorCanMsgInterval);
	}
}
