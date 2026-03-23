#pragma once

// #include <Arduino.h>
#include <cstring>
#include <cmath>
#include <cstdlib>

struct GpsSnapshot {
	bool timeValid;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint16_t milliseconds;

	bool dateValid;
	uint16_t year;
	uint8_t month;
	uint8_t day;

	bool locationValid;
	double latitudeDeg;
	double longitudeDeg;

	bool altitudeValid;
	double altitudeMeters;

	bool speedValid;
	double speedKmh;

	bool courseValid;
	double courseDeg;

	bool satellitesValid;
	uint8_t satellites;

	bool hdopValid;
	float hdop;

	bool vdopValid;
	float vdop;
};

uint16_t encodeRcGpsAltitude(double meters);
uint16_t encodeRcGpsSpeed(double kmh);

uint32_t buildRcGpsTimeField(uint8_t syncBits);