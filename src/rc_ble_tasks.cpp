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
				"RaceChrono filter updated: allow all: interval=%u ms\n",
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
			g_canFilterState.requestedPids[g_canFilterState.requestedPidCount].nextDueMs = 0;
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
	static size_t allowAllCursor = 0;
	static size_t requestedPidCursor = 0;
	while (true) {
		if (!isRaceChronoClientConnected()) {
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}

		uint32_t now = millis();

		bool allowAll = false;
		uint16_t allowAllIntervalMs = 0;
		RequestedPid requestedPids[kMaxRequestedPids]{};
		size_t requestedPidCount = 0;

		// Snapshot current filter state
		if (xSemaphoreTake(g_canFilterState.mutex, portMAX_DELAY) == pdTRUE) {
			allowAll = g_canFilterState.allowAll;
			allowAllIntervalMs = g_canFilterState.allowAllIntervalMs;
			requestedPidCount = g_canFilterState.requestedPidCount;
			if (requestedPidCount > kMaxRequestedPids) {
				requestedPidCount = kMaxRequestedPids;
			}

			for (size_t i = 0; i < requestedPidCount; ++i) {
				requestedPids[i] = g_canFilterState.requestedPids[i];
			}

			xSemaphoreGive(g_canFilterState.mutex);
		}

		if (!allowAll && requestedPidCount == 0) {
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}

		bool hasFrameToSend = false;
		uint32_t framePid;
		uint8_t frameData[8];
		size_t selectedRequestedPidIndex = kMaxRequestedPids;

		// Snapshot/select from cache
		if (xSemaphoreTake(g_canFrameCache.mutex, portMAX_DELAY) == pdTRUE) {
			if (allowAll) {
				if (g_canFrameCache.count > 0) {
					if (allowAllCursor >= g_canFrameCache.count) {
						allowAllCursor = 0;
					}

					// Walk forward until a valid cached frame is found, wrapping if needed.
					for (size_t checked = 0; checked < g_canFrameCache.count; ++checked) {
						const size_t index = (allowAllCursor + checked) % g_canFrameCache.count;
						if (!g_canFrameCache.entries[index].valid) {
							continue;
						}

						framePid = g_canFrameCache.entries[index].identifier;
						memcpy(frameData, g_canFrameCache.entries[index].data, 8);
						hasFrameToSend = true;
						allowAllCursor = (index + 1) % g_canFrameCache.count;
						break;
					}
				}
			} else {
				// Round-robin from the cursor and emit the first PID that is due and present in cache.
				for (size_t checkedPid = 0; checkedPid < requestedPidCount; ++checkedPid) {
					const size_t pidIndex = (requestedPidCursor + checkedPid) % requestedPidCount;
					const RequestedPid& requested = requestedPids[pidIndex];

					if (!requested.active) {
						continue;
					}

					if (requested.intervalMs > 0 && now < requested.nextDueMs) {
						continue;
					}

					for (size_t cacheIndex = 0; cacheIndex < g_canFrameCache.count; ++cacheIndex) {
						if (!g_canFrameCache.entries[cacheIndex].valid) {
							continue;
						}

						if (g_canFrameCache.entries[cacheIndex].identifier != requested.pid) {
							continue;
						}

						framePid = g_canFrameCache.entries[cacheIndex].identifier;
						memcpy(frameData, g_canFrameCache.entries[cacheIndex].data, 8);
						hasFrameToSend = true;
						selectedRequestedPidIndex = pidIndex;
						requestedPidCursor = (pidIndex + 1) % requestedPidCount;
						break;
					}

					if (hasFrameToSend) {
						break;
					}
				}
			}

			xSemaphoreGive(g_canFrameCache.mutex);
		}

		if (hasFrameToSend) {
			uint8_t payload[13]{};
			// 4-byte CAN identifier, little-endian
			payload[0] = static_cast<uint8_t>(framePid & 0xFF);
			payload[1] = static_cast<uint8_t>((framePid >> 8) & 0xFF);
			payload[2] = static_cast<uint8_t>((framePid >> 16) & 0xFF);
			payload[3] = static_cast<uint8_t>((framePid >> 24) & 0xFF);

			// 8 data bytes
			memcpy(&payload[4], frameData, 8);

			g_mainChar->setValue(payload, sizeof(payload));
			g_mainChar->notify();

			// Update due time for the PID we just sent.
			if (!allowAll && selectedRequestedPidIndex < requestedPidCount) {
				if (xSemaphoreTake(g_canFilterState.mutex, portMAX_DELAY) == pdTRUE) {
					if (selectedRequestedPidIndex < g_canFilterState.requestedPidCount) {
						g_canFilterState.requestedPids[selectedRequestedPidIndex].nextDueMs =
							now + g_canFilterState.requestedPids[selectedRequestedPidIndex].intervalMs;
					}
					xSemaphoreGive(g_canFilterState.mutex);
				}
			}
		}

		vTaskDelayUntil(&lastWake, NotifyInterval);
	}
}