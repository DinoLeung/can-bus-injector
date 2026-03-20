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

/**
 * @brief FreeRTOS task responsible for applying incoming RaceChrono filter requests.
 *
 * This task blocks on `g_rcPidFilterRequestQueue` and updates the shared
 * RaceChrono PID filter state whenever the client sends a new filter command.
 * Supported commands are:
 * - `DenyAll`: disable allow-all mode and clear the requested PID list.
 * - `AllowAll`: enable allow-all mode with the requested interval and clear
 *   any previously requested specific PIDs.
 * - `AllowOnePid`: disable allow-all mode and append a single requested PID
 *   with its transmission interval to the request list.
 *
 * State updates are protected by the filter state's internal mutex so the
 * notify task can safely snapshot and consume the configuration from another
 * FreeRTOS task.
 *
 * Scheduling notes:
 * - Newly added specific PIDs are initialized with `nextDueMs = millis()` so
 *   they are eligible for transmission immediately.
 * - The task itself is event-driven and does not perform periodic polling;
 *   it wakes only when a new queue entry is received.
 *
 * @param pvParameters Unused FreeRTOS task parameter.
 */
static void raceChronoCanFilterRequestTask(void* pvParameters) {
	(void)pvParameters;
	TickType_t lastWake = xTaskGetTickCount();
	RcFilterRequest request{};

	while (true) {
		if (xQueueReceive(g_rcPidFilterRequestQueue, &request, portMAX_DELAY) != pdTRUE) {
			continue;
		}

		if (xSemaphoreTake(g_rcPidFilterState.mutex, portMAX_DELAY) != pdTRUE) {
			continue;
		}

		switch (request.command) {
		case RcFilterCommand::DenyAll:
			g_rcPidFilterState.allowAll = false;
			g_rcPidFilterState.allowAllIntervalMs = 0;
			g_rcPidFilterState.requestedPidCount = 0;
			for (size_t i = 0; i < kMaxRequestedPids; ++i) {
				g_rcPidFilterState.requestedPids[i] = RequestedPid{};
			}
			Serial.println("RaceChrono filter updated: deny all");
			break;

		case RcFilterCommand::AllowAll:
			g_rcPidFilterState.allowAll = true;
			g_rcPidFilterState.allowAllIntervalMs = request.intervalMs;
			g_rcPidFilterState.requestedPidCount = 0;
			Serial.printf(
				"RaceChrono filter updated: allow all: interval=%u ms\n",
				static_cast<unsigned>(request.intervalMs));
			break;

		case RcFilterCommand::AllowOnePid:
			g_rcPidFilterState.allowAll = false;
			g_rcPidFilterState.allowAllIntervalMs = 0;
			if (g_rcPidFilterState.requestedPidCount >= kMaxRequestedPids) {
				Serial.println("RaceChrono filter request ignored: requested PID list full");
				break;
			}

			g_rcPidFilterState.requestedPids[g_rcPidFilterState.requestedPidCount].pid = request.pid;
			g_rcPidFilterState.requestedPids[g_rcPidFilterState.requestedPidCount].active = true;
			g_rcPidFilterState.requestedPids[g_rcPidFilterState.requestedPidCount].intervalMs = request.intervalMs;
			g_rcPidFilterState.requestedPids[g_rcPidFilterState.requestedPidCount].nextDueMs = millis();
			++g_rcPidFilterState.requestedPidCount;
			Serial.printf(
				"RaceChrono filter updated: allow pid=0x%08lX interval=%u ms\n",
				static_cast<unsigned long>(request.pid),
				static_cast<unsigned>(request.intervalMs));
			break;
		}
		xSemaphoreGive(g_rcPidFilterState.mutex);
	}
}

/**
 * @brief FreeRTOS task responsible for streaming CAN frames over BLE to RaceChrono.
 *
 * This task runs at a fixed interval (~200 Hz) and orchestrates the selection
 * and transmission of CAN frames based on the current filter configuration.
 *
 * High-level flow per iteration:
 * 1. Check BLE connection state and early-exit if disconnected.
 * 2. Snapshot the current filter configuration from `CanFilterState` to avoid
 *    holding locks during processing.
 * 3. Depending on mode:
 *    - Allow-all: round-robin through the cache using `allowAllCursor`.
 *    - Specific PIDs: select the next due PID via `nextDuePid()` and fetch
 *      its cached frame.
 * 4. If a frame is available, pack it into RaceChrono's 13-byte payload format
 *    (4-byte identifier + 8-byte data + implicit DLC=8) and notify via BLE.
 * 5. For specific PID mode, update scheduling state using `markPidSent()`.
 *
 * Concurrency model:
 * - Filter state and cache access are internally synchronized via mutexes.
 * - This task only operates on snapshots or uses thread-safe accessors.
 *
 * Scheduling notes:
 * - `allowAllCursor` and `requestedPidCursor` maintain fairness across frames
 *   and requested PIDs respectively.
 * - The task interval (`NotifyInterval`) effectively sets the minimum gap
 *   between BLE notifications.
 *
 * @param pvParameters Unused FreeRTOS task parameter.
 */
static void raceChronoCanNotifyTask(void* pvParameters) {
	(void)pvParameters;
	TickType_t lastWake = xTaskGetTickCount();
	static size_t allowAllCursor = 0;
	static size_t requestedPidCursor = 0;
	while (true) {
		if (!g_rcBleConnected) {
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}

		uint32_t now = millis();
		bool allowAll = false;
		uint16_t allowAllIntervalMs = 0;
		RequestedPid requestedPids[kMaxRequestedPids]{};
		size_t requestedPidCount = 0;

		// Snapshot current filter state
		g_rcPidFilterState.snapshot(
			allowAll,
			allowAllIntervalMs,
			requestedPids,
			requestedPidCount);

		if (!allowAll && requestedPidCount <= 0) {
			// Tick longer to free up cpu time
			vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
			continue;
		}

		bool hasFrameToSend = false;
		uint32_t framePid;
		uint8_t frameData[8];
		size_t selectedRequestedPidIndex = kMaxRequestedPids;

		// Snapshot cache allow all mode
		if (allowAll) {
			hasFrameToSend = g_canFrameCache.getNextCachedFrame(allowAllCursor, framePid, frameData);
		}

		// Snapshot cache specific PIDs mode
		if (requestedPidCount > 0) {
			RequestedPid duePid;
			bool hasDue = g_rcPidFilterState.nextDuePid(
				now,
				requestedPidCursor,
				duePid,
				selectedRequestedPidIndex);

			if (hasDue) {
				hasFrameToSend = g_canFrameCache.getNextRequestedCachedFrame(
					duePid,
					framePid,
					frameData);
			}
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

			g_rcBleMainChar->setValue(payload, sizeof(payload));
			g_rcBleMainChar->notify();

			// Update due time for the PID we just sent.
			if (!allowAll && selectedRequestedPidIndex < requestedPidCount) {
				g_rcPidFilterState.markPidSent(selectedRequestedPidIndex, now);
			}
		}

		vTaskDelayUntil(&lastWake, NotifyInterval);
	}
}