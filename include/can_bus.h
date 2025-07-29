#pragma once
#include <driver/twai.h>
#include <mcp_can.h>
#include <SPI.h>

// Initialize CAN1 (TWAI) in listen-only
bool initCan1();

// Initialize CAN2 (MCP2515) in normal mode
bool initCan2();

// Send a TWAI message over CAN2
bool writeCan2(const twai_message_t& message);
