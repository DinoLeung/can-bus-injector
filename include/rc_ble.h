#pragma once

#include <stdint.h>
#include "FreeRTOS.h"
#include "freertos/semphr.h"
#include <BLE2902.h>
#include <BLEDevice.h>
#include <freertos/queue.h>

static constexpr const char* kDeviceName = "CAN Pulse BLE";
static constexpr uint16_t kServiceUuid = 0x1FF8;
static constexpr uint16_t kCanMainCharUuid = 0x0001;
static constexpr uint16_t kCanFilterCharUuid = 0x0002;
static constexpr uint16_t kGpsMainCharUuid = 0x0003;
static constexpr uint16_t kGpsTimeCharUuid = 0x0004;

static constexpr size_t kMaxRequestedPids = 64;
static constexpr size_t kFilterRequestQueueSize = 16;

struct RequestedPid {
	uint32_t pid;
	uint16_t intervalMs;
	uint32_t nextDueMs;
	bool active;
};

struct CanFilterState {
	bool allowAll;
	uint16_t allowAllIntervalMs;
	RequestedPid requestedPids[kMaxRequestedPids];
	size_t requestedPidCount;
	SemaphoreHandle_t mutex;
};

extern CanFilterState g_canFilterState;
extern QueueHandle_t g_filterRequestQueue;

extern BLEServer* g_server;
extern BLECharacteristic* g_mainChar;
extern volatile bool g_connected;

bool initRaceChronoBle();
bool isRaceChronoClientConnected();
void raceChronoStartAdvertising();