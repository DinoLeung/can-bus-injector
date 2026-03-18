#include <Arduino.h>
#include "can_bus.h"
#include "can_frame_cache.h"
#include "rc_ble.h"
#include "sensor_tasks.h"
#include "can_tasks.h"
#include "rc_ble_tasks.h"

void setup() {
	Serial.begin(115200);

	initCanFrameCache();

	initCan1();
	// initCan2();
	initRaceChronoBle();
	
	startCanTasks();
	// startSensorTasks();
	startRaceChronoTasks();
}

void loop() { /* DO NOTHING */ }
