#pragma once

#include <Arduino.h>

enum class RcFilterCommand : uint8_t {
	DenyAll = 0,
	AllowAll = 1,
	AllowOnePid = 2,
};

struct RcFilterRequest {
	RcFilterCommand command;
	uint16_t intervalMs;
	uint32_t pid;
};

bool parseFilterRequest(const std::string& value, RcFilterRequest& out);