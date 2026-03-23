#include "gps_helper.h"

#include <cstdlib>
#include <TinyGPSPlus.h>

#include "gps.h"

/**
 * @brief Encodes RaceChrono GPS altitude field.
 *
 * Uses the 0.1 m encoding when the value fits in the range [-500, 6053.5] meters,
 * Otherwise fall back to 1 m precision with (((meters + 500) & 0x7FFF) | 0x8000). Invalid value is 0xFFFF.
 */
uint16_t encodeRcGpsAltitude(double meters) {
	if (!std::isfinite(meters)) {
		return 0xFFFF;
	}

	const double shifted = meters + 500.0;
	const long fineEncoded = lround(shifted * 10.0);

	if (fineEncoded >= 0 && fineEncoded <= 0x7FFF) {
		return static_cast<uint16_t>(fineEncoded & 0x7FFF);
	}

	const long coarseEncoded = lround(shifted);
	if (coarseEncoded >= 0 && coarseEncoded <= 0x7FFF) {
		return static_cast<uint16_t>((coarseEncoded & 0x7FFF) | 0x8000);
	}

	return 0xFFFF;
}

/**
 * @brief Encodes RaceChrono GPS speed field from km/h.
 *
 * Uses the 0.01 km/h encoding when the value fits in range [0, 655.35] km/h,
 * otherwise falls back to the 0.1 km/h encoding with the top bit set.
 */
uint16_t encodeRcGpsSpeed(double kmh) {
	if (!std::isfinite(kmh) || kmh < 0.0) {
		return 0xFFFF;
	}

	const long fineEncoded = lround(kmh * 100.0);
	if (fineEncoded >= 0 && fineEncoded <= 0x7FFF) {
		return static_cast<uint16_t>(fineEncoded & 0x7FFF);
	}

	const long coarseEncoded = lround(kmh * 10.0);
	if (coarseEncoded >= 0 && coarseEncoded <= 0x7FFF) {
		return static_cast<uint16_t>((coarseEncoded & 0x7FFF) | 0x8000);
	}

	return 0xFFFF;
}

/**
 * @brief Encodes RaceChrono GPS bearing field from degrees.
 *
 * Normalizes the input into the range [0, 360) and encodes it as
 * degrees * 100. Invalid value is 0xFFFF.
 */
uint16_t encodeRcGpsBearing(double degrees) {
	if (!std::isfinite(degrees)) {
		return 0xFFFF;
	}

	double bearing = std::fmod(degrees, 360.0);
	if (bearing < 0.0) {
		bearing += 360.0;
	}

	return static_cast<uint16_t>(lround(bearing * 100.0));
}

/**
 * @brief Encodes a DOP value for the 8-bit RaceChrono GPS HDOP/VDOP fields.
 *
 * Encodes the value as dop * 10. The maximum valid encoded value is 0xFE,
 * while 0xFF is reserved as the invalid sentinel.
 */
uint8_t encodeRcGpsDop(float dop) {
	if (!std::isfinite(dop) || dop < 0.0f) {
		return 0xFF;
	}

	int encoded = static_cast<int>(lround(dop * 10.0f));
	if (encoded > 0xFE) {
		encoded = 0xFE;
	}

	return static_cast<uint8_t>(encoded);
}

/**
 * @brief Maps TinyGPS fix quality values into the 2-bit RaceChrono fix quality field.
 *
 * RaceChrono only provides 2 bits for fix quality in the GPS main payload:
 * - 0: invalid / no fix
 * - 1: GPS fix (2D)
 * - 2: GPS fix (3D)
 * - 3: DGPS-class fix
 *
 * TinyGPS exposes a broader NMEA-derived quality enum, so this helper compresses
 * those source values into the closest RaceChrono representation:
 * - Invalid -> 0
 * - GPS -> 2
 * - DGPS, PPS, RTK, FloatRTK -> 3
 * - Estimated, Manual, Simulated -> 0
 *
 * TinyGPS does not provide enough information here to distinguish ordinary 2D
 * GPS from ordinary 3D GPS, so the plain GPS quality is mapped to the 3D code.
 *
 * @param q TinyGPS location quality enum value.
 * @return Encoded 2-bit RaceChrono fix quality value in the range [0, 3].
 */
uint8_t encodeRcFixQuality(TinyGPSLocation::Quality q) {
	switch (q) {
	case TinyGPSLocation::Quality::Invalid:
		return 0;

	case TinyGPSLocation::Quality::GPS:
		return 2;

	case TinyGPSLocation::Quality::DGPS:
	case TinyGPSLocation::Quality::PPS:
	case TinyGPSLocation::Quality::RTK:
	case TinyGPSLocation::Quality::FloatRTK:
		return 3;

	case TinyGPSLocation::Quality::Estimated:
	case TinyGPSLocation::Quality::Manual:
	case TinyGPSLocation::Quality::Simulated:
	default:
		return 0;
	}
}
