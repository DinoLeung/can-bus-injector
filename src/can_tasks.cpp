#include "can_tasks.h"
#include "can_bus.h"
#include "globals.h"

// Custom CAN ID
#define CAN_ID 0x600
// message queue for can forwarding task
static QueueHandle_t messageQueue = nullptr;

constexpr TickType_t sensorCanMsgInterval = pdMS_TO_TICKS(50);

static void readCan1EnqueueTask(void*);
static void forwardCan1ToCan2Task(void*);
static void sensorCanWriterTask(void*);

void startCanTasks() {
	messageQueue = xQueueCreate(10, sizeof(twai_message_t));
	xTaskCreate(readCan1EnqueueTask, "CAN1_Read", 4096, NULL, 1, NULL);
	xTaskCreate(forwardCan1ToCan2Task, "CAN1_Read", 4096, NULL, 1, NULL);
	xTaskCreate(sensorCanWriterTask, "Sensor_CAN_Writer", 2048, NULL, 1, NULL);
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
void readCan1EnqueueTask(void* pvParameters) {
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
		taskYIELD();
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
void forwardCan1ToCan2Task(void* pvParameters) {
	(void)pvParameters;
	twai_message_t message;
	while (true) {
		if (xQueueReceive(messageQueue, &message, portMAX_DELAY) == pdTRUE) {
			writeCan2(message);
		}
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
