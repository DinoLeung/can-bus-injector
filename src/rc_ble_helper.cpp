#include "rc_ble_helper.h"
#include <Arduino.h>

static uint16_t readBe16(const uint8_t* data) {
	return (static_cast<uint16_t>(data[0]) << 8) |
	       static_cast<uint16_t>(data[1]);
}

static uint32_t readBe32(const uint8_t* data) {
	return (static_cast<uint32_t>(data[0]) << 24) |
	       (static_cast<uint32_t>(data[1]) << 16) |
	       (static_cast<uint32_t>(data[2]) << 8) |
	       static_cast<uint32_t>(data[3]);
}

bool parseFilterRequest(const std::string& value, RcFilterRequest& out) {
	if (value.empty()) {
		return false;
	}

	const uint8_t* data = reinterpret_cast<const uint8_t*>(value.data());
	const size_t len = value.length();
	const uint8_t command = data[0];

	switch (command) {
	case 0: // deny all
		if (len != 1) return false;
		out.command = RcFilterCommand::DenyAll;
		out.intervalMs = 0;
		out.pid = 0;
		return true;

	case 1: // allow all
		if (len != 3) return false;
		out.command = RcFilterCommand::AllowAll;
		out.intervalMs = readBe16(&data[1]);
		out.pid = 0;
		return true;

	case 2: // allow one pid
		if (len != 7) return false;
		out.command = RcFilterCommand::AllowOnePid;
		out.intervalMs = readBe16(&data[1]);
		out.pid = readBe32(&data[3]);
		return true;

	default:
		return false;
	}
}