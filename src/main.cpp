#include <Arduino.h>
#include <driver/twai.h>
#include <mcp_can.h>
#include <SPI.h>

#define LED_1 GPIO_NUM_2

// CAN1 (TWAI) Pins
#define CAN1_RX_PIN GPIO_NUM_6
#define CAN1_TX_PIN GPIO_NUM_7

// CAN2 (MCP2515) Custom SPI Pins
#define CAN2_CS_PIN GPIO_NUM_10
#define CAN2_SPI_SCK GPIO_NUM_12
#define CAN2_SPI_MISO GPIO_NUM_13
#define CAN2_SPI_MOSI GPIO_NUM_11

// HSPI bus
SPIClass CAN2_SPI(HSPI);
MCP_CAN CAN2(&CAN2_SPI, CAN2_CS_PIN);

// Solder on pins
#define OIL_PRESURE GPIO_NUM_NC
#define OIL_TEMP GPIO_NUM_NC

// put function declarations here:
bool setupCAN1();
bool setupCAN2();

void setup() {
	// put your setup code here, to run once:
	// pinMode(LED_1, OUTPUT);
}

void loop() {
	// digitalWrite(LED_1, HIGH);
	// delay(1000);
	// digitalWrite(LED_1, LOW);
	// delay(1000);
}

// put function definitions here:
bool setupCAN1()
{
	twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN1_TX_PIN, (gpio_num_t)CAN1_RX_PIN, TWAI_MODE_NORMAL);
	twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
	twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

	if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
		Serial.println("Failed to install TWAI driver");
		return false;
	}

	if (twai_start() != ESP_OK) {
		Serial.println("Failed to start TWAI driver");
		return false;
	}

	Serial.println("CAN1 (TWAI) started. Waiting for messages...");
	return true;
}

bool setupCAN2()
{
	CAN2_SPI.begin(CAN2_SPI_SCK, CAN2_SPI_MISO, CAN2_SPI_MOSI, CAN2_CS_PIN);

	if (CAN2.begin(MCP_STDEXT, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {
		Serial.println("Failed to start MCP2515 driver");
		return false;
	}
	
	CAN2.setMode(MCP_NORMAL);
	Serial.println("CAN2 (MCP2515) started...");
	return true;
}