#include <Arduino.h>
#include "can_frame_cache.h"
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
	CanFrameCacheEntry entries[capacity]{};
	size_t count = 0;
	// size_t nextInsertIndex;
	SemaphoreHandle_t mutex = nullptr;
};

/**
 * @brief Global CAN frame cache shared across tasks.
 *
 * This object holds all cached CAN frames and the mutex used to protect
 * concurrent access.
 */
CanFrameCache g_canFrameCache;

/**
 * @brief Find an existing cache entry by CAN identifier and frame format.
 *
 * Performs a linear search through the active portion of the cache and returns
 * the first valid entry whose identifier and standard/extended flag match the
 * requested key.
 *
 * @param identifier CAN identifier to search for.
 * @param isExtended True for extended 29-bit frames, false for standard 11-bit frames.
 * @return Pointer to the matching cache entry, or nullptr if no entry exists.
 */
static CanFrameCacheEntry* findCacheEntry(uint32_t identifier, bool isExtended) {
	for (size_t i = 0; i < g_canFrameCache.count; i++) {
		CanFrameCacheEntry& entry = g_canFrameCache.entries[i];
		if (entry.valid && entry.identifier == identifier && entry.isExtended == isExtended) {
			return &entry;
		}
	}
	return nullptr;
}

/**
 * @brief Initialize the global CAN frame cache.
 *
 * Creates the cache mutex on first use, resets the entry count, and clears all
 * cache slots back to an empty state.
 */
void initCanFrameCache() {
	if (g_canFrameCache.mutex == nullptr) {
		g_canFrameCache.mutex = xSemaphoreCreateMutex();
	}
	g_canFrameCache.count = 0;
	for (size_t i = 0; i < g_canFrameCache.capacity; i++) {
		g_canFrameCache.entries[i].valid = false;
		g_canFrameCache.entries[i].lastUpdatedMs = 0;
		g_canFrameCache.entries[i].dlc = 0;
		g_canFrameCache.entries[i].identifier = 0;
		g_canFrameCache.entries[i].isExtended = false;
		memset(g_canFrameCache.entries[i].data, 0, sizeof(g_canFrameCache.entries[i].data));
	}
}

/**
 * @brief Insert or update a cached CAN frame.
 *
 * Looks up an existing cache entry for the given CAN identifier and frame
 * format. If none exists, a new entry is allocated from the next free slot.
 * The payload bytes, DLC, and last-seen timestamp are then updated while the
 * cache mutex is held.
 *
 * @param identifier CAN identifier of the received frame.
 * @param isExtended True for extended 29-bit frames, false for standard 11-bit frames.
 * @param dlc Data length code of the received frame. Values greater than 8 are clamped to 8.
 * @param data Pointer to the frame payload bytes. May be nullptr when dlc is 0.
 * @return true if the cache entry was updated successfully, or false if the mutex could not be taken or the cache is full.
 */
bool updateCanFrameCache(uint32_t identifier, bool isExtended, uint8_t dlc, const uint8_t* data) {
	if (xSemaphoreTake(g_canFrameCache.mutex, portMAX_DELAY) != pdTRUE) {
		return false;
	}

	CanFrameCacheEntry* entry = findCacheEntry(identifier, isExtended);
	if (entry == nullptr) {
		if (g_canFrameCache.count >= g_canFrameCache.capacity) {
			xSemaphoreGive(g_canFrameCache.mutex);
			return false;
		}
		entry = &g_canFrameCache.entries[g_canFrameCache.count++];
		entry->identifier = identifier;
		entry->isExtended = isExtended;
	}

	entry->dlc = dlc > 8 ? 8 : dlc;
	memset(entry->data, 0, sizeof(entry->data));
	if (data != nullptr && entry->dlc > 0) {
		memcpy(entry->data, data, entry->dlc);
	}
	entry->lastUpdatedMs = millis();
	entry->valid = true;

	xSemaphoreGive(g_canFrameCache.mutex);
	return true;
}