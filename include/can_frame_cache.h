#pragma once
#include <cstdint>

void initCanFrameCache();
bool updateCanFrameCache(uint32_t identifier, bool isExtended, uint8_t dlc, const uint8_t* data);