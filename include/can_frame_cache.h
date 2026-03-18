#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

void initCanFrameCache();
bool updateCanFrameCache(uint32_t identifier, bool isExtended, uint8_t dlc, const uint8_t* data);