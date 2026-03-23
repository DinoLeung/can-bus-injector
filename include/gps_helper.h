#pragma once

#include <cstdlib>
#include <TinyGPSPlus.h>

uint16_t encodeRcGpsAltitude(double meters);
uint16_t encodeRcGpsSpeed(double kmh);
uint16_t encodeRcGpsBearing(double degrees);
uint8_t encodeRcGpsDop(float dop);
uint8_t encodeRcFixQuality(TinyGPSLocation::Quality q);
