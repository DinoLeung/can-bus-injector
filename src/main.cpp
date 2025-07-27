#include <Arduino.h>
#include <driver/twai.h>
#include <mcp_can.h>
#include <SPI.h>
#include <math.h>
#include <functional>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Sensor readings
volatile uint16_t g_oilPressurePsi10 = 0;
volatile uint16_t g_oilTempC10 = 0;

#define LED_1 GPIO_NUM_2

// CAN1 (TWAI) Pins
#define CAN1_RX_PIN GPIO_NUM_6
#define CAN1_TX_PIN GPIO_NUM_7

// CAN2 (MCP2515) Custom SPI Pins
#define CAN2_CS_PIN GPIO_NUM_10
#define CAN2_SPI_SCK GPIO_NUM_12
#define CAN2_SPI_MISO GPIO_NUM_13
#define CAN2_SPI_MOSI GPIO_NUM_11

// HSPI bus
SPIClass CAN2_SPI(HSPI);
MCP_CAN CAN2(&CAN2_SPI, CAN2_CS_PIN);

// PST-F1 signal pins
#define OIL_PRESSURE GPIO_NUM_35
#define OIL_TEMP GPIO_NUM_36

// Custom CAN ID
#define CAN_ID 0x600

// Analog Digital Converter const
constexpr float VREF = 3.3f;
constexpr float ADC_MAX = 4095.0f;

// message queue for can forwarding task
static QueueHandle_t messageQueue = nullptr;

// put function declarations here:
bool setupCan1();
bool setupCan2();
bool writeCan2(twai_message_t message);
void can1ReadTask(void* pvParameters);
void can1ForwardTask(void* pvParameters);
void oilPressureTask(void* pvParameters);
void oilTempTask(void* pvParameters);
void sensorCanWriterTask(void* pvParameters);

void setup() {
	Serial.begin(115200);
	setupCan1();
	setupCan2();
	// create queue to hold up to 10 CAN frames
	messageQueue = xQueueCreate(10, sizeof(twai_message_t));
	// Configure ADC resolution and attenuation for analog sensors
	analogReadResolution(12);
	analogSetPinAttenuation(OIL_PRESSURE, ADC_11db);
	analogSetPinAttenuation(OIL_TEMP, ADC_11db);
	// start FreeRTOS tasks
	xTaskCreate(can1ReadTask, "CAN1_Read", 4096, NULL, 1, NULL);
	xTaskCreate(can1ForwardTask, "CAN1_Forward", 4096, NULL, 1, NULL);
	xTaskCreate(oilPressureTask, "Oil_Pres", 2048, NULL, 1, NULL);
	xTaskCreate(oilTempTask, "Oil_Temp", 2048, NULL, 1, NULL);
	xTaskCreate(sensorCanWriterTask, "Sensor_CAN_Writer", 2048, NULL, 1, NULL);
}

void loop() { /* DO NOTHING */ }

/**
 * @brief Convert raw ADC reading to voltage (0–3.3V).
 * @param raw ADC count (0–4095).
 * @return Voltage corresponding to raw ADC value.
 */
static float rawToVoltage(int raw) {
	return raw * VREF / ADC_MAX;
}

/**
 * @brief Initialize and start the TWAI (CAN1) driver on the configured pins.
 *
 * This function installs the TWAI driver using the default general, timing,
 * and filter configurations (normal mode, 250 kbit/s, accept all filters),
 * then starts the driver. It logs success or failure messages to Serial.
 *
 * @return true if the driver was installed and started successfully.
 * @return false if driver installation or startup fails.
 */
bool setupCan1() {
	twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN1_TX_PIN, (gpio_num_t)CAN1_RX_PIN, TWAI_MODE_NORMAL);
	twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
	twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

	if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
		Serial.println("Failed to install TWAI driver");
		return false;
	}

	if (twai_start() != ESP_OK) {
		Serial.println("Failed to start TWAI driver");
		return false;
	}

	Serial.println("CAN1 (TWAI) started. Waiting for messages...");
	return true;
}

/**
 * @brief Initialize and start the MCP2515 (CAN2) driver over the custom SPI interface.
 *
 * This function configures the HSPI bus and initializes the MCP2515 controller
 * at 1 Mbit/s with a 16 MHz oscillator, then sets it to normal operation mode.
 * It logs success or failure messages to Serial.
 *
 * @return true if the MCP2515 driver was initialized and set to normal mode successfully.
 * @return false if initialization or mode setting fails.
 */
bool setupCan2() {
	CAN2_SPI.begin(CAN2_SPI_SCK, CAN2_SPI_MISO, CAN2_SPI_MOSI, CAN2_CS_PIN);

	if (CAN2.begin(MCP_STDEXT, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {
		Serial.println("Failed to start MCP2515 driver");
		return false;
	}
	
	CAN2.setMode(MCP_NORMAL);
	Serial.println("CAN2 (MCP2515) started...");
	return true;
}

/**
 * @brief Send a CAN frame out over the MCP2515-based CAN2 bus.
 *
 * Logs the outgoing frame identifier, type (standard or extended), and data payload to Serial,
 * then transmits it using the MCP_CAN interface.
 *
 * @param message The TWAI message structure containing identifier, data length, and payload bytes.
 * @return true if the frame was transmitted successfully (CAN_OK).
 * @return false if transmission failed, logging the error code to Serial.
 */
bool writeCan2(twai_message_t message) {
	// Serial.printf("Sending message to CAN2 [0x%03lX]: ", can_id);
	Serial.printf("Sending message to CAN2 [0x%08lX %s]: ", message.identifier, message.extd ? "EXT" : "STD");
	for (byte i = 0; i < message.data_length_code; i++)
	{
	Serial.printf("0x%02X", message.data[i]);
	if (i < message.data_length_code - 1)
		Serial.print(", ");
	}
	Serial.println();

	// Send the message via CAN2
	byte result = CAN2.sendMsgBuf(message.identifier, message.extd, message.data_length_code, message.data);

	if (result == CAN_OK) {
		Serial.println("Message sent successfully via CAN2");
		return true;
	} else {
		Serial.printf("Message send failed, code: %d\n", result);
		return false;
	}
}

/**
 * @brief FreeRTOS task that polls CAN1 (TWAI) for incoming frames and enqueues them.
 *
 * This task continuously reads available CAN1 messages using `twai_receive()`.
 * For each received frame, it logs the identifier, frame type (standard or extended),
 * and payload bytes to Serial, then sends the `twai_message_t` into the global
 * FreeRTOS queue `messageQueue`. After draining the buffer, it delays for one tick
 * to yield CPU time to other tasks.
 */
void can1ReadTask(void* pvParameters) {
	(void)pvParameters;
	twai_message_t message;
	while (true) {
		while (twai_receive(&message, 0) == ESP_OK) {
			// Print original CAN1 message
			// Serial.printf("[CAN1 0x%X]: ", message.identifier);
			Serial.printf("[CAN1 0x%X %s]: ", message.identifier, message.extd ? "EXT" : "STD");
			for (int i = 0; i < message.data_length_code; i++) {
				Serial.printf("0x%02X", message.data[i]);
				if (i < message.data_length_code - 1)
					Serial.print(", ");
			}
			Serial.println();
	
			// emit message into RTOS queue
			xQueueSend(messageQueue, &message, portMAX_DELAY);
		}
		vTaskDelay(1);
	}
}

/**
 * @brief FreeRTOS task that forwards CAN1 messages from the queue to CAN2.
 *
 * This task blocks on the global FreeRTOS queue `messageQueue` until a
 * `twai_message_t` arrives. Upon receiving a message, it invokes
 * `writeCan2()` to transmit the frame on the MCP2515-based CAN2 interface.
 * The task runs in an infinite loop to continuously handle incoming frames.
 *
 * @param pvParameters Unused parameter for FreeRTOS compatibility.
 */
void can1ForwardTask(void* pvParameters) {
	(void)pvParameters;
	twai_message_t message;
	while (true) {
		if (xQueueReceive(messageQueue, &message, portMAX_DELAY) == pdTRUE) {
			writeCan2(message);
		}
	}
}

/**
 * @brief FreeRTOS task that samples the analog voltage from the oil pressure sensor,
 *        computes actual sensor voltage, and calculates oil pressure in bar and psi.
 *
 * Reads ADC from OIL_PRESSURE pin, converts to voltage, undoes voltage divider,
 * converts to pressure (bar and psi), and update `g_oilPressurePsi10` every 50 ms (20 Hz).
 */
void oilPressureTask(void* pvParameters) {
	(void)pvParameters;
	// Voltage divider resistors: Rtop = 5.6 kΩ, Rbot = 10 kΩ
	constexpr float DIVIDER_RATIO = 10.0f / (5.6f + 10.0f);
	// Sensor characteristic: 0.5V @ 0 bar, 4.5V @ 10 bar
	constexpr float ZERO_VOLTAGE = 0.5f;
	constexpr float FULL_SCALE_VOLTAGE = 0.5f;
	constexpr float SENSITIVITY_BAR_PER_VOLT = 10.0f / (FULL_SCALE_VOLTAGE - ZERO_VOLTAGE); // 2.5 bar/V
	constexpr float BAR_TO_PSI = 14.5038f;

	while (true) {
		int raw = analogRead(OIL_PRESSURE);
		float vAdc = rawToVoltage(raw);
		// Compute actual sensor voltage before divider
		float vSensor = vAdc / DIVIDER_RATIO;
		// Calculate pressure in bar
		float pressureBar = (vSensor - ZERO_VOLTAGE) * SENSITIVITY_BAR_PER_VOLT;
		// Convert to psi
		float pressurePsi = pressureBar * BAR_TO_PSI;

		Serial.printf(
			"Oil Pressure raw: %d, Vadc: %.3f V, Vsensor: %.3f V, Pressure: %.2f bar (%.2f psi)\n",
			raw, vAdc, vSensor, pressureBar, pressurePsi
		);

		g_oilPressurePsi10 = static_cast<uint16_t>(pressurePsi * 10.0f);

		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

/**
 * @brief FreeRTOS task that samples the analog voltage from the oil temperature NTC sensor,
 *        computes sensor resistance, and converts it to temperature in °C.
 *
 * Reads ADC from OIL_TEMP pin, converts to voltage, computes sensor resistance via divider equation,
 * converts resistance to temperature using the Beta parameter method, and update `g_oilTempC10` every 50 ms (20 Hz).
 *
 */
void oilTempTask(void* pvParameters) {
	(void)pvParameters;
	// Divider top resistor: 30 kΩ
	constexpr float R_TOP = 30000.0f;
	// Calibration points for Beta calculation
	// -40°C: R = 44 kΩ
	// -40°C = 233.15K
	constexpr float T1 = 233.15f;
	constexpr float R1 = 44000.0f;
	// 150°C: R = 58 Ω
	// 150°C = 423.15K
	constexpr float T2 = 423.15f;
	constexpr float R2 = 58.0f;
	// Compute Beta constant
	const float BETA = log(R1 / R2) / ((1.0f / T1) - (1.0f / T2));

	while (true) {
		int raw = analogRead(OIL_TEMP);
		float vAdc = rawToVoltage(raw);
		// Compute sensor resistance from divider equation: Vout = VREF * (R_sensor/(R_top + R_sensor))
		float rSensor = R_TOP * (vAdc / (VREF - vAdc));
		// Compute temperature via Beta formula: 1/T = 1/T1 + (1/BETA)*ln(R/R1)
		float invT = (1.0f / T1) + (log(rSensor / R1) / BETA);
		float tempK = 1.0f / invT;
		float tempC = tempK - 273.15f;

		Serial.printf(
			"Oil Temp raw: %d, Vadc: %.3f V, Rsen: %.1f Ω, Temp: %.2f °C\n",
			raw, vAdc, rSensor, tempC
		);

		g_oilTempC10 = static_cast<uint16_t>(tempC * 10.0f);

		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

/**
 * @brief FreeRTOS task that periodically sends combined sensor values to CAN2.
 *
 * This task constructs a 4-byte CAN frame every 50 ms (20 Hz) that includes:
 * - Bytes 0–1: Oil pressure in deci-psi (uint16_t)
 * - Bytes 2–3: Oil temperature in deci-degrees Celsius (uint16_t)
 *
 * The values are read from shared volatile variables `g_oilPressurePsi10` and `g_oilTempC10`,
 * and sent as a standard 11-bit CAN frame using the identifier defined by `CAN_ID`.
 *
 * @param pvParameters Unused parameter for FreeRTOS compatibility.
 */
void sensorCanWriterTask(void* pvParameters) {
	(void)pvParameters;
	while (true) {
		twai_message_t msg{};
		msg.identifier = CAN_ID;
		msg.extd = 0;
		msg.data_length_code = 4;
		msg.data[0] = uint8_t(g_oilPressurePsi10 >> 8);
		msg.data[1] = uint8_t(g_oilPressurePsi10 & 0xFF);
		msg.data[2] = uint8_t(g_oilTempC10 >> 8);
		msg.data[3] = uint8_t(g_oilTempC10 & 0xFF);
		writeCan2(msg);
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}
