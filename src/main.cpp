#include <Arduino.h>
#include "can_bus.h"
#include "can_frame_cache.h"
#include "gps.h"
#include "rc_ble.h"
#include "sensor_tasks.h"
#include "can_tasks.h"
#include "gps_tasks.h"
#include "rc_ble_tasks.h"

void setup() {
	initCanFrameCache();

	initCanBus();
	initGps();
	initRaceChronoBle();
	
	startCanTasks();
	startGpsTasks();
	// startSensorTasks();
	startRaceChronoTasks();
}

void loop() { /* DO NOTHING */ }
