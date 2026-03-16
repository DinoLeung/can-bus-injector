#include "racechrono.h"
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>

namespace {
	constexpr const char* kDeviceName = "CAN Pulse";
	constexpr uint16_t kServiceUuid = 0x1FF8;
	constexpr uint16_t kCanMainCharUuid = 0x0001;
	constexpr uint16_t kCanFilterCharUuid = 0x0002;
	constexpr uint16_t kGpsMainCharUuid = 0x0003;
	constexpr uint16_t kGpsTimeCharUuid = 0x0004;

	BLEServer* g_server = nullptr;
	BLECharacteristic* g_mainChar = nullptr;
	volatile bool g_connected = false;

	static BLEService* createRaceChronoService();
	static void createCanMainCharacteristic(BLEService* service);
	static void createCanFilterCharacteristic(BLEService* service);
	static void startRaceChronoAdvertising();


	/**
	 * @brief BLE server callbacks used to track RaceChrono connection state.
	 *
	 * This class hooks into the ESP32 BLE server lifecycle events. When the
	 * RaceChrono app connects, `onConnect()` sets the global connection flag so
	 * other tasks know a client is present. When the client disconnects,
	 * `onDisconnect()` clears the flag and restarts advertising so the device
	 * becomes discoverable again.
	 */
	class ServerCallbacks : public BLEServerCallbacks {
		public:
		void onConnect(BLEServer*) override {
			g_connected = true;
			Serial.println("RaceChrono connected");
		}

		void onDisconnect(BLEServer* server) override {
			g_connected = false;
			Serial.println("RaceChrono disconnected");
			server->getAdvertising()->start();
		}
	};

	/**
	 * @brief Callback handler for the RaceChrono CAN filter characteristic.
	 *
	 * RaceChrono writes commands to characteristic 0x0002 to control which CAN
	 * frames the device should transmit over BLE. For now this implementation
	 * only logs the write length, but later this will parse filter commands such
	 * as "allow PID", "deny all", or "allow all" with a requested interval.
	 */
	class FilterCallbacks : public BLECharacteristicCallbacks {
		public:
		void onWrite(BLECharacteristic* characteristic) override {
			auto value = characteristic->getValue();
			Serial.printf("RaceChrono filter write, len=%u\n", static_cast<unsigned>(value.length()));
			Serial.print("RaceChrono filter raw: ");
			for (auto b : value)
				Serial.printf("%02X ", (uint8_t)b);
			Serial.println();
		}
	};

	/**
	 * @brief Create the primary RaceChrono BLE service.
	 *
	 * This helper allocates the GATT service that hosts the RaceChrono
	 * characteristics used by the mobile app.
	 *
	 * @return Pointer to the created BLE service.
	 */
	static BLEService* createRaceChronoService() {
		return g_server->createService(BLEUUID(kServiceUuid));
	}

	/**
	 * @brief Create the RaceChrono CAN main characteristic.
	 *
	 * This characteristic is used for RaceChrono data notifications. It is
	 * configured with READ and NOTIFY properties and initialised with a small
	 * zeroed value so the attribute exists with known contents before the first
	 * real payload is sent.
	 *
	 * @param service The RaceChrono BLE service that will own the characteristic.
	 */
	static void createCanMainCharacteristic(BLEService* service) {
		g_mainChar = service->createCharacteristic(
			BLEUUID(kCanMainCharUuid),
			BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
		g_mainChar->addDescriptor(new BLE2902());

		uint8_t initValue[4] = {0, 0, 0, 0};
		g_mainChar->setValue(initValue, sizeof(initValue));
	}

	/**
	 * @brief Create the RaceChrono CAN filter characteristic.
	 *
	 * RaceChrono writes filter commands to this characteristic to describe which
	 * CAN identifiers it wants to receive and at what interval.
	 *
	 * @param service The RaceChrono BLE service that will own the characteristic.
	 */
	static void createCanFilterCharacteristic(BLEService* service) {
		auto* filterChar = service->createCharacteristic(
			BLEUUID(kCanFilterCharUuid),
			BLECharacteristic::PROPERTY_WRITE);
		filterChar->setCallbacks(new FilterCallbacks());
	}

	/**
	 * @brief Start BLE advertising for the RaceChrono service.
	 *
	 * This makes the device discoverable to the RaceChrono mobile app using the
	 * service UUID configured for the DIY BLE device profile.
	 */
	static void startRaceChronoAdvertising() {
		auto* advertising = BLEDevice::getAdvertising();
		advertising->addServiceUUID(BLEUUID(kServiceUuid));
		BLEDevice::startAdvertising();
	}
}

/**
 * @brief Initialise the RaceChrono BLE service and start advertising.
 *
 * This function sets up the ESP32 BLE stack, creates the RaceChrono GATT
 * service (UUID 0x1FF8), and registers the required characteristics:
 *
 * - 0x0001 : CAN main characteristic (READ + NOTIFY)
 * - 0x0002 : CAN filter characteristic (WRITE)
 *
 * After the service is started the device begins advertising so the
 * RaceChrono mobile app can discover and connect to it.
 *
 * @return true when BLE initialisation completes successfully.
 */
bool initRaceChronoBle() {
	BLEDevice::init(kDeviceName);

	g_server = BLEDevice::createServer();
	g_server->setCallbacks(new ServerCallbacks());

	auto* service = createRaceChronoService();
	createCanMainCharacteristic(service);
	createCanFilterCharacteristic(service);

	service->start();
	startRaceChronoAdvertising();

	Serial.println("RaceChrono BLE ready");
	return true;
}

/**
 * @brief Returns whether a RaceChrono client is currently connected.
 *
 * Other tasks (for example CAN streaming tasks) can call this helper to
 * determine whether BLE notifications should be sent. If no client is
 * connected there is no reason to push data.
 *
 * @return true if the RaceChrono app is connected to the BLE server.
 */
bool isRaceChronoClientConnected() {
	return g_connected;
}

/**
 * @brief Restart BLE advertising for the RaceChrono service.
 *
 * This helper simply restarts advertising using the previously configured
 * service UUID. It can be used if advertising needs to be restarted after
 * a disconnect or after BLE has been temporarily stopped.
 */
void raceChronoStartAdvertising() {
	BLEDevice::startAdvertising();
}