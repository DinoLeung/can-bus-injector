#include <Arduino.h>
#include "can_bus.h"
#include "racechrono.h"
#include "sensor_tasks.h"
#include "can_tasks.h"
#include "racechrono_tasks.h"

void setup() {
	Serial.begin(115200);

	// initCan1();
	// initCan2();
	initRaceChronoBle();
	
	// startCanTasks();
	// startSensorTasks();
	startRaceChronoTasks();
}

void loop() { /* DO NOTHING */ }
