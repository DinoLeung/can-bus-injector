#include "gps_helper.h"
#include <Arduino.h>

#include "gps.h"
#include "rc_ble_helper.h"

/**
 * @brief Encodes RaceChrono GPS altitude field.
 *
 * Uses the 0.1 m encoding when the value fits in the documented range,
 * otherwise falls back to the 1 m encoding with the top bit set.
 */
uint16_t encodeRcGpsAltitude(double meters) {
	if (!std::isfinite(meters)) {
		return 0xFFFF;
	}

	if (meters >= -500.0 && meters <= 6053.5) {
		const long encoded = lround((meters + 500.0) * 10.0);
		return static_cast<uint16_t>(encoded & 0x7FFF);
	}

	const long encoded = lround(meters + 500.0);
	return static_cast<uint16_t>((encoded & 0x7FFF) | 0x8000);
}

/**
 * @brief Encodes RaceChrono GPS speed field from km/h.
 *
 * Uses the 0.01 km/h encoding when the value fits in range, otherwise falls
 * back to the 0.1 km/h encoding with the top bit set.
 */
uint16_t encodeRcGpsSpeed(double kmh) {
	if (!std::isfinite(kmh) || kmh < 0.0) {
		return 0xFFFF;
	}

	if (kmh <= 655.35) {
		const long encoded = lround(kmh * 100.0);
		return static_cast<uint16_t>(encoded & 0x7FFF);
	}

	const long encoded = lround(kmh * 10.0);
	return static_cast<uint16_t>((encoded & 0x7FFF) | 0x8000);
}

/**
 * @brief Builds the 24-bit RaceChrono GPS time characteristic payload.
 */
uint32_t buildRcGpsTimeField(uint8_t syncBits) {
	if (!g_gps.date.isValid() || !g_gps.time.isValid()) {
		return static_cast<uint32_t>(syncBits & 0x07) << 21;
	}

	const uint32_t year = static_cast<uint32_t>(g_gps.date.year() - 2000);
	const uint32_t month = static_cast<uint32_t>(g_gps.date.month() - 1);
	const uint32_t day = static_cast<uint32_t>(g_gps.date.day() - 1);
	const uint32_t hour = static_cast<uint32_t>(g_gps.time.hour());
	const uint32_t hourAndDate = year * 8928U + month * 744U + day * 24U + hour;
	return (static_cast<uint32_t>(syncBits & 0x07) << 21) | (hourAndDate & 0x1FFFFF);
}
