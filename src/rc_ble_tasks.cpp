#include "rc_ble_tasks.h"
#include <Arduino.h>
#include "rc_ble.h"
#include "rc_ble_helper.h"
#include "can_frame_cache.h"

// 200Hz
constexpr TickType_t NotifyInterval = pdMS_TO_TICKS(5);

static void raceChronoCanFilterRequestTask(void*);
static void raceChronoCanNotifyTask(void*);

void startRaceChronoTasks() {
	xTaskCreate(raceChronoCanFilterRequestTask, "RaceChronoNotify", 4096, nullptr, 1, nullptr);
	xTaskCreate(raceChronoCanNotifyTask, "RaceChronoNotify", 4096, nullptr, 1, nullptr);
}

static void raceChronoCanFilterRequestTask(void* pvParameters) {
	(void)pvParameters;
	TickType_t lastWake = xTaskGetTickCount();
	RcFilterRequest request{};

	while (true) {
		if (xQueueReceive(g_filterRequestQueue, &request, portMAX_DELAY) != pdTRUE) {
			continue;
		}

		if (xSemaphoreTake(g_canFilterState.mutex, portMAX_DELAY) != pdTRUE) {
			continue;
		}

		switch (request.command) {
		case RcFilterCommand::DenyAll:
			g_canFilterState.allowAll = false;
			g_canFilterState.allowAllIntervalMs = 0;
			g_canFilterState.requestedPidCount = 0;
			for (size_t i = 0; i < kMaxRequestedPids; ++i) {
				g_canFilterState.requestedPids[i] = RequestedPid{};
			}
			Serial.println("RaceChrono filter updated: deny all");
			break;

		case RcFilterCommand::AllowAll:
			g_canFilterState.allowAll = true;
			g_canFilterState.allowAllIntervalMs = request.intervalMs;
			g_canFilterState.requestedPidCount = 0;
			Serial.printf(
				"RaceChrono filter updated: allow all, interval=%u ms\n",
				static_cast<unsigned>(request.intervalMs));
			break;

		case RcFilterCommand::AllowOnePid:
			g_canFilterState.allowAll = false;
			g_canFilterState.allowAllIntervalMs = 0;
			if (g_canFilterState.requestedPidCount >= kMaxRequestedPids) {
				Serial.println("RaceChrono filter request ignored: requested PID list full");
				break;
			}

			g_canFilterState.requestedPids[g_canFilterState.requestedPidCount].pid = request.pid;
			g_canFilterState.requestedPids[g_canFilterState.requestedPidCount].intervalMs = request.intervalMs;
			++g_canFilterState.requestedPidCount;
			Serial.printf(
				"RaceChrono filter updated: allow pid=0x%08lX interval=%u ms\n",
				static_cast<unsigned long>(request.pid),
				static_cast<unsigned>(request.intervalMs));
			break;
		}
		xSemaphoreGive(g_canFilterState.mutex);
	}
}

static void raceChronoCanNotifyTask(void* pvParameters) {
	(void)pvParameters;
	TickType_t lastWake = xTaskGetTickCount();
	while (true) {
		if (!isRaceChronoClientConnected()) {
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}

		uint32_t now = millis();

		bool allowAll = false;
		uint16_t allowAllIntervalMs = 0;
		// RequestedPid requestedPids[64];
		size_t requestedPidCount = 0;

		// snapshot filter mode
		if (xSemaphoreTake(g_canFilterState.mutex, portMAX_DELAY) == pdTRUE) {
			allowAll = g_canFilterState.allowAll;
			allowAllIntervalMs = g_canFilterState.allowAllIntervalMs;
			requestedPidCount = g_canFilterState.requestedPidCount;
			xSemaphoreGive(g_canFilterState.mutex);
		}

		if (allowAll == false && requestedPidCount == 0) {
			vTaskDelay(pdMS_TO_TICKS(1000));
			continue;
		}

		// snapshot requested PIDs
		if (xSemaphoreTake(g_canFilterState.mutex, portMAX_DELAY) == pdTRUE) {
			
			xSemaphoreGive(g_canFilterState.mutex);
		}

		// snapshot cache
		if (xSemaphoreTake(g_canFrameCache.mutex, portMAX_DELAY) == pdTRUE) {

			xSemaphoreGive(g_canFrameCache.mutex);
		}

		if (allowAll) {
			// iterate cache
		} else {
			// iterate requestedPids
		}

		vTaskDelayUntil(&lastWake, NotifyInterval);
	}
}