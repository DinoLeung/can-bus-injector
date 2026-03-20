#include "can_frame_cache.h"
#include <Arduino.h>

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

/**
 * @brief Retrieve the next cached CAN frame in allow-all mode.
 *
 * Iterates through the cache in a round-robin fashion using the provided
 * cursor, returning the next valid cached frame. The cursor is advanced
 * to ensure fair traversal across all cached entries over time.
 *
 * The operation is protected by the cache mutex to ensure safe concurrent
 * access from multiple tasks.
 *
 * @param cursor          In/out cursor used for round-robin traversal.
 * @param outFramePid     Output CAN identifier of the selected frame.
 * @param outFrameData    Output buffer (8 bytes) containing frame payload.
 * @return true if a valid frame was found and returned, false otherwise.
 */
bool CanFrameCache::getNextCachedFrame(
	size_t& cursor,
	uint32_t& outFramePid,
	uint8_t (&outFrameData)[8]) const {
	if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
		return false;
	}

	bool found = false;

	if (count > 0) {
		if (cursor >= count) {
			cursor = 0;
		}

		for (size_t checked = 0; checked < count; ++checked) {
			const size_t index = (cursor + checked) % count;
			if (!entries[index].valid) {
				continue;
			}

			outFramePid = entries[index].identifier;
			memcpy(outFrameData, entries[index].data, 8);
			cursor = (index + 1) % count;
			found = true;
			break;
		}
	}

	xSemaphoreGive(mutex);
	return found;
}

/**
 * @brief Retrieve the cached CAN frame for a requested PID.
 *
 * Looks up the cache for a valid frame matching the requested PID and, if
 * found, copies the cached identifier and payload into the caller-provided
 * outputs.
 *
 * Due-time and round-robin scheduling decisions are handled by
 * `PidFilterState::nextDuePid()`. This function is only responsible for
 * cache lookup.
 *
 * The operation is protected by the cache mutex to ensure safe concurrent
 * access from multiple tasks.
 *
 * @param requestedPid   Requested PID to look up in the cache.
 * @param outFramePid    Output CAN identifier of the selected frame.
 * @param outFrameData   Output buffer (8 bytes) containing frame payload.
 * @return true if a matching cached frame was found and returned, false otherwise.
 */
bool CanFrameCache::getNextRequestedCachedFrame(
	const RequestedPid& requestedPid,
	uint32_t& outFramePid,
	uint8_t (&outFrameData)[8]) const {
	if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
		return false;
	}

	bool found = false;

	for (size_t cacheIndex = 0; cacheIndex < count; ++cacheIndex) {
		if (!entries[cacheIndex].valid) {
			continue;
		}

		if (entries[cacheIndex].identifier != requestedPid.pid) {
			continue;
		}

		outFramePid = entries[cacheIndex].identifier;
		memcpy(outFrameData, entries[cacheIndex].data, 8);
		found = true;
		break;
	}

	xSemaphoreGive(mutex);
	return found;
}