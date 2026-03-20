#include "rc_ble_helper.h"
#include <Arduino.h>

/**
 * @brief Read a 16-bit big-endian unsigned integer from a byte buffer.
 *
 * Interprets the first two bytes at the given pointer as a big-endian
 * encoded 16-bit value (network byte order) and converts it to host order.
 *
 * @param data Pointer to at least 2 bytes of data.
 * @return Decoded 16-bit unsigned integer.
 */
static uint16_t readBe16(const uint8_t* data) {
	return (static_cast<uint16_t>(data[0]) << 8) |
	       static_cast<uint16_t>(data[1]);
}

/**
 * @brief Read a 32-bit big-endian unsigned integer from a byte buffer.
 *
 * Interprets the first four bytes at the given pointer as a big-endian
 * encoded 32-bit value (network byte order) and converts it to host order.
 *
 * @param data Pointer to at least 4 bytes of data.
 * @return Decoded 32-bit unsigned integer.
 */
static uint32_t readBe32(const uint8_t* data) {
	return (static_cast<uint32_t>(data[0]) << 24) |
	       (static_cast<uint32_t>(data[1]) << 16) |
	       (static_cast<uint32_t>(data[2]) << 8) |
	       static_cast<uint32_t>(data[3]);
}

/**
 * @brief Parse a RaceChrono BLE filter request payload.
 *
 * Decodes a raw byte payload received from the RaceChrono client into a
 * structured `RcFilterRequest`. The payload format is command-based:
 *
 * - Command 0 (DenyAll):
 *   - Length: 1 byte
 *   - Disables all streaming.
 *
 * - Command 1 (AllowAll):
 *   - Length: 3 bytes
 *   - [1..2]: 16-bit big-endian interval in milliseconds
 *   - Enables streaming of all cached frames at the given interval.
 *
 * - Command 2 (AllowOnePid):
 *   - Length: 7 bytes
 *   - [1..2]: 16-bit big-endian interval in milliseconds
 *   - [3..6]: 32-bit big-endian CAN identifier (PID)
 *   - Requests streaming of a specific PID at the given interval.
 *
 * Invalid payload lengths or unknown command values will result in failure.
 *
 * @param value Raw payload received over BLE (binary string).
 * @param out   Output structure populated on successful parse.
 * @return true if parsing succeeded, false otherwise.
 */
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