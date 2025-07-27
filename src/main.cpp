#include <Arduino.h>
#include <driver/twai.h>
#include <mcp_can.h>
#include <SPI.h>
#include <vector>
#include <functional>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

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
#define OIL_PRESURE GPIO_NUM_35
#define OIL_TEMP GPIO_NUM_36

// message queue for can forwarding task
static QueueHandle_t messageQueue = nullptr;

// put function declarations here:
bool setupCan1();
bool setupCan2();
bool writeCan2(twai_message_t message);
void can1ReadTask(void* pvParameters);
void can1ProcessTask(void* pvParameters);


void setup() {
	Serial.begin(115200);
    setupCan1();
    setupCan2();
    // create queue to hold up to 10 CAN frames
	messageQueue = xQueueCreate(10, sizeof(twai_message_t));
    // start FreeRTOS tasks
    xTaskCreate(can1ReadTask, "CAN1_Read", 4096, NULL, 1, NULL);
    xTaskCreate(can1ProcessTask, "CAN1_Forward", 4096, NULL, 1, NULL);
}

void loop() { /* DO NOTHING */ }

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
 * @brief FreeRTOS task that processes CAN1 messages from the queue and forwards them on CAN2.
 *
 * This task blocks on the global FreeRTOS queue `messageQueue` until a
 * `twai_message_t` arrives. Upon receiving a message, it invokes
 * `writeCan2()` to transmit the frame on the MCP2515-based CAN2 interface.
 * The task runs in an infinite loop to continuously handle incoming frames.
 *
 * @param pvParameters Unused parameter for FreeRTOS compatibility.
 */
void can1ProcessTask(void* pvParameters) {
    twai_message_t message;
    while (true) {
        if (xQueueReceive(messageQueue, &message, portMAX_DELAY) == pdTRUE) {
            writeCan2(message);
        }
    }
}

