#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/**
 * @brief Single cached CAN frame entry.
 *
 * Stores the most recently observed payload for a specific CAN identifier and
 * frame format combination. Each entry represents one logical frame source in
 * the cache and keeps enough metadata for consumers to determine whether the
 * slot is populated and how fresh the data is.
 */
struct CanFrameCacheEntry {
	uint32_t identifier;
	bool isExtended;
	uint8_t dlc;
	uint8_t data[8];
	uint32_t lastUpdatedMs;
	bool valid;
};

/**
 * @brief Fixed-capacity global cache for the latest CAN frames.
 *
 * The cache is implemented as a static array to avoid heap allocations at
 * runtime. Access is protected by a FreeRTOS mutex because the cache is shared
 * across multiple tasks.
 */
struct CanFrameCache {
	static constexpr size_t capacity = 256;
	CanFrameCacheEntry entries[capacity];
	size_t count;
	SemaphoreHandle_t mutex;
};

extern CanFrameCache g_canFrameCache;

void initCanFrameCache();
bool updateCanFrameCache(uint32_t identifier, bool isExtended, uint8_t dlc, const uint8_t* data);