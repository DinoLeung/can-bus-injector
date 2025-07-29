#include <Arduino.h>
#include "can_bus.h"
#include "sensor_tasks.h"
#include "can_tasks.h"

void setup() {
	Serial.begin(115200);

	initCan1();
	initCan2();
	
	startCanTasks();
	startSensorTasks();
}

void loop() { /* DO NOTHING */ }
