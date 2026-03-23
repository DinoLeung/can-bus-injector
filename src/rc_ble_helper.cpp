#include "rc_ble_helper.h"
// #include <Arduino.h>
#include <cstring>
#include <cmath>
#include <cstdlib>

#include "gps.h"
#include "gps_helper.h"

/**
 * @brief Read a 16-bit big-endian unsigned integer from a byte buffer.
 *
 * Interprets the first two bytes at the given pointer as a big-endian
 * encoded 16-bit value (network byte order) and converts it to host order.
 *
 * @param data Pointer to at least 2 bytes of data.
 * @return Decoded 16-bit unsigned integer.
 */
uint16_t readBe16(const uint8_t* data) {
	return (static_cast<uint16_t>(data[0]) << 8) |
	       static_cast<uint16_t>(data[1]);
}

/**
 * @brief Write a 16-bit unsigned integer in big-endian byte order.
 */
void writeBe16(const uint16_t value, uint8_t* output) {
	output[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
	output[1] = static_cast<uint8_t>(value & 0xFF);
}

/**
 * @brief Write a 24-bit unsigned integer in big-endian byte order.
 */
void writeBe24(const uint32_t value, uint8_t* output) {
	output[0] = static_cast<uint8_t>((value >> 16) & 0xFF);
	output[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
	output[2] = static_cast<uint8_t>(value & 0xFF);
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
uint32_t readBe32(const uint8_t* data) {
	return (static_cast<uint32_t>(data[0]) << 24) |
	       (static_cast<uint32_t>(data[1]) << 16) |
	       (static_cast<uint32_t>(data[2]) << 8) |
	       static_cast<uint32_t>(data[3]);
}

/**
 * @brief Write a 32-bit signed integer in big-endian byte order.
 */
void writeBe32(const int32_t value, uint8_t* output) {
	const uint32_t raw = static_cast<uint32_t>(value);
	output[0] = static_cast<uint8_t>((raw >> 24) & 0xFF);
	output[1] = static_cast<uint8_t>((raw >> 16) & 0xFF);
	output[2] = static_cast<uint8_t>((raw >> 8) & 0xFF);
	output[3] = static_cast<uint8_t>(raw & 0xFF);
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

void buildRcCanMainPayload(uint32_t framePid, const uint8_t* frameData, uint8_t* outPayload) {
	memset(outPayload, 0, 13);

	// 4-byte CAN identifier, little-endian
	outPayload[0] = static_cast<uint8_t>(framePid & 0xFF);
	outPayload[1] = static_cast<uint8_t>((framePid >> 8) & 0xFF);
	outPayload[2] = static_cast<uint8_t>((framePid >> 16) & 0xFF);
	outPayload[3] = static_cast<uint8_t>((framePid >> 24) & 0xFF);

	// 8 data bytes
	memcpy(&outPayload[4], frameData, 8);
}

/**
 * @brief Builds the 20-byte RaceChrono GPS main characteristic payload.
 */
void buildRcGpsMainPayload(uint8_t syncBits, uint8_t* outPayload) {
	memset(outPayload, 0, 20);

	// Byte 0-2 Sync bits* (3 bits) and time from hour start (21 bits = (minute * 30000) + (seconds * 500) + (milliseconds / 2))
	uint32_t timeFromHour = 0;
	if (g_gps.time.isValid()) {
		timeFromHour =
			static_cast<uint32_t>(g_gps.time.minute()) * 30000U +
			static_cast<uint32_t>(g_gps.time.second()) * 500U +
			static_cast<uint32_t>(g_gps.time.centisecond()) * 5U;
	}
	writeBe24((static_cast<uint32_t>(syncBits & 0x07) << 21) | (timeFromHour & 0x1FFFFF), outPayload);

	// Byte 3 Fix quality (2 bits), locked satellites (6 bits, invalid value 0x3F)
	uint8_t fixSat = 0;
	if (g_gps.location.isValid()) {
		const uint8_t fixQuality = 1;
		uint8_t satellites = 0x3F;
		if (g_gps.satellites.isValid()) {
			const uint32_t satValue = g_gps.satellites.value();
			satellites = static_cast<uint8_t>(satValue > 0x3F ? 0x3F : satValue);
		}
		fixSat = static_cast<uint8_t>((fixQuality << 6) | (satellites & 0x3F));
	} else {
		fixSat = 0x3F;
	}
	outPayload[3] = fixSat;

	// Byte 4-7 Latitude in (degrees * 10_000_000), signed 2's complement, invalid value 0x7FFFFFFF
	// Byte 8-11 Longitude in (degrees * 10_000_000), signed 2's complement, invalid value 0x7FFFFFFF
	if (g_gps.location.isValid()) {
		const int32_t latitude = static_cast<int32_t>(llround(g_gps.location.lat() * 10000000.0));
		const int32_t longitude = static_cast<int32_t>(llround(g_gps.location.lng() * 10000000.0));
		writeBe32(latitude, &outPayload[4]);
		writeBe32(longitude, &outPayload[8]);
	} else {
		writeBe32(0x7FFFFFFF, &outPayload[4]);
		writeBe32(0x7FFFFFFF, &outPayload[8]);
	}

	// Byte 12-13 Altitude (((meters + 500) * 10) & 0x7FFF) or (((meters + 500) & 0x7FFF) | 0x8000), invalid value 0xFFFF. **
	const uint16_t altitude = g_gps.altitude.isValid()
		? encodeRcGpsAltitude(g_gps.altitude.meters())
		: 0xFFFF;
	writeBe16(altitude, &outPayload[12]);

	// Byte 14-15 Speed in ((km/h * 100) & 0x7FFF) or (((km/h * 10) & 0x7FFF) | 0x8000), invalid value 0xFFFF.
	const uint16_t speed = g_gps.speed.isValid()
		? encodeRcGpsSpeed(g_gps.speed.kmph())
		: 0xFFFF;
	writeBe16(speed, &outPayload[14]);

	// Byte 16-17 Bearing (degrees * 100), invalid value 0xFFFF
	const uint16_t bearing = g_gps.course.isValid()
		? static_cast<uint16_t>(lround(g_gps.course.deg() * 100.0))
		: 0xFFFF;
	writeBe16(bearing, &outPayload[16]);

	// Byte 18 HDOP (dop * 10), invalid value 0xFF
	outPayload[18] = g_gps.hdop.isValid()
		? static_cast<uint8_t>(lround(g_gps.hdop.hdop() * 10.0))
		: 0xFF;
	// Byte 19 VDOP (dop * 10), invalid value 0xFF
	if (g_vdop.isValid()) {
		float vdop = atof(g_vdop.value());
		outPayload[19] = static_cast<uint8_t>(lround(vdop * 10.0));
	} else {
		outPayload[19] = 0xFF;
	}
}