#include <Arduino.h>

#define LED_1 GPIO_NUM_2

// CAN1 (TWAI) Pins
#define CAN1_RX_PIN GPIO_NUM_6
#define CAN1_TX_PIN GPIO_NUM_7

// CAN2 (MCP2515) Custom SPI Pins
#define CAN2_CS_PIN GPIO_NUM_10
#define CAN2_SPI_SCK GPIO_NUM_12
#define CAN2_SPI_MISO GPIO_NUM_13
#define CAN2_SPI_MOSI GPIO_NUM_11

// Solder on pins
#define OIL_PRESURE GPIO_NUM_NC
#define OIL_TEMP GPIO_NUM_NC

// put function declarations here:
// int myFunction(int, int);

void setup() {
	// put your setup code here, to run once:
	// int result = myFunction(2, 3);
	pinMode(LED_1, OUTPUT);
}

void loop() {
	// put your main code here, to run repeatedly:
	digitalWrite(LED_1, HIGH);
	delay(1000);
	digitalWrite(LED_1, LOW);
	delay(1000);
}

// put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }