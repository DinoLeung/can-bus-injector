#include "racechrono_tasks.h"
#include "racechrono.h"
#include <Arduino.h>

// 1Hz
constexpr TickType_t HeartbeatInterval = pdMS_TO_TICKS(1000);

static void raceChronoHeartbeatTask(void*);

void startRaceChronoTasks() {
	xTaskCreate(raceChronoHeartbeatTask, "RaceChrono", 4096, nullptr, 1, nullptr);
}

static void raceChronoHeartbeatTask(void* pvParameters) {
	(void)pvParameters;
	TickType_t lastWake = xTaskGetTickCount();
	while (true) {
		// if (isRaceChronoClientConnected()) {
		// 	Serial.println("RaceChrono client is connected");
		// }
		vTaskDelayUntil(&lastWake, HeartbeatInterval);
	}
}