#pragma once
#include <driver/twai.h>

// Initialize CAN1 (TWAI) in listen-only
// Initialize CAN2 (MCP2515) in normal mode
bool initCanBus();

// Send a TWAI message over CAN2
bool writeCan2(const twai_message_t& message);
