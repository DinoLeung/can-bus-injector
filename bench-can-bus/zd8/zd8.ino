#include <Arduino.h>
#include "driver/twai.h"

// BRZ bus speed from the original example
static constexpr uint32_t CAN_BITRATE = 500000;

// -------------------------------------------------------------------------------------------------
// Frame scheduler
// -------------------------------------------------------------------------------------------------

struct ScheduledPid {
  uint16_t pid;
  uint16_t rate_hz;
  uint32_t last_sent_us;
};

// Keep the same spirit as the original example:
// - 0x18, 0x140, 0x141, 0x142 at 100 Hz
// - several others at 50 / 20 / 10 / 16.7 Hz
ScheduledPid scheduledPids[] = {
  {0x018, 100, 0},
  // {0x140, 100, 0},
  {0x141, 100, 0},
  {0x142, 100, 0},
  {0x040, 100, 0},

  {0x138,  50, 0},
  {0x139,  50, 0},
  {0x13B,  50, 0},
  {0x146,  50, 0},
  {0x241,  50, 0},
  {0x345,  50, 0},
  {0x390,  50, 0},
  // {0x0D0,  50, 0},
  // {0x0D1,  50, 0},
  // {0x0D2,  50, 0},
  // {0x0D3,  50, 0},
  // {0x0D4,  50, 0},
  {0x144,  50, 0},
  {0x152,  50, 0},
  {0x156,  50, 0},
  {0x280,  50, 0},

  {0x282,  17, 0},  // original note: ~16.7 Hz
  {0x284,  10, 0},
  {0x360,  20, 0},
};

// -------------------------------------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------------------------------------

static inline uint16_t hzToPeriodUs(uint16_t hz) {
  return 1000000UL / hz;
}

// bool sendFrame(uint16_t id, const uint8_t *payload, uint8_t len) {
//   twai_message_t msg = {};
//   msg.identifier = id;
//   msg.extd = 0;  // standard 11-bit frame
//   msg.rtr = 0;
//   msg.ss = 0;
//   msg.self = 0;
//   msg.dlc_non_comp = 0;
//   msg.data_length_code = len;

//   memcpy(msg.data, payload, len);

//   esp_err_t err = twai_transmit(&msg, 0);
//   return err == ESP_OK;
// }

bool sendFrame(uint16_t id, const uint8_t *payload, uint8_t len) {
  twai_message_t msg = {};
  msg.identifier = id;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = len;
  memcpy(msg.data, payload, len);

  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(50));
  if (err != ESP_OK) {
    Serial.printf("TX failed id=0x%03X err=%d\n", id, err);
    return false;
  }
  return true;
}

// -------------------------------------------------------------------------------------------------
// Fake BRZ payload generation
// -------------------------------------------------------------------------------------------------

void generatePayload(uint16_t pid, uint8_t *payload) {
  memset(payload, 0, 8);

  switch (pid) {
    case 0x018: {
      // Placeholder heartbeat frame.
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0x00;
      payload[3] = 0x00;
      payload[4] = 0x00;
      payload[5] = 0x00;
      payload[6] = 0x00;
      payload[7] = 0x00;
      break;
    }

    case 0x040: {
      // engine speed / accelerator pos
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0xCF;
      payload[3] = 0x82;
      payload[4] = 0x99;
      payload[5] = 0x00;
      payload[6] = 0x00;
      payload[7] = 0x00;
      break;
    }

    case 0x138: {
      // Steering / yaw
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0xC1;
      payload[3] = 0xFF;
      payload[4] = 0xFA;
      payload[5] = 0x3F;
      payload[6] = 0x00;
      payload[7] = 0x00;
      break;
    }

    case 0x139: {
      // Speed / break pressure
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0xFC;
      payload[3] = 0xE0;
      payload[4] = 0x00;
      payload[5] = 0x01;
      payload[6] = 0x00;
      payload[7] = 0x00;
      break;
    }

    case 0x13B: {
      // acceleration
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0x00;
      payload[3] = 0x00;
      payload[4] = 0x00;
      payload[5] = 0x0;
      payload[6] = 0xFE;
      payload[7] = 0xFD;
      break;
    }

    case 0x146: {
      // acceleration
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0x00;
      payload[3] = 0x00;
      payload[4] = 0x89;
      payload[5] = 0x00;
      payload[6] = 0x00;
      payload[7] = 0x00;
      break;
    }

    case 0x241: {
      // acceleration
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0x00;
      payload[3] = 0x00;
      payload[4] = 0x30;
      payload[5] = 0x00;
      payload[6] = 0x00;
      payload[7] = 0x00;
      break;
    }

    case 0x345: {
      // acceleration
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0x00;
      payload[3] = 0x84;
      payload[4] = 0x82;
      payload[5] = 0x12;
      payload[6] = 0x00;
      payload[7] = 0x00;
      break;
    }
    
    case 0x390: {
      // acceleration
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0x00;
      payload[3] = 0x00;
      payload[4] = 0x90;
      payload[5] = 0x00;
      payload[6] = 0x00;
      payload[7] = 0x00;
      break;
    }

    // case 0x0D0: {
    //   // Steering / yaw / acceleration
    //   int16_t steering_angle_degrees = -123;
    //   int16_t steering_value = steering_angle_degrees * 10;
    //   payload[0] = steering_value & 0xFF;
    //   payload[1] = (steering_value >> 8) & 0xFF;

    //   int16_t rotation_clockwise_deg_s = -70;
    //   int16_t rotation_value = (int16_t)(-3.14159f * rotation_clockwise_deg_s);
    //   payload[2] = rotation_value & 0xFF;
    //   payload[3] = (rotation_value >> 8) & 0xFF;

    //   float lateral_accel_g = 0.3f;
    //   payload[6] = (int8_t)(9.80665f * lateral_accel_g / 0.2f);

    //   float longitudinal_accel_g = 0.2f;
    //   payload[7] = (int8_t)(-9.80665f * longitudinal_accel_g / 0.1f);
    //   break;
    // }

    // case 0x0D1: {
    //   // Speed + brake pressure
    //   //
    //   // The original example appears to overwrite payload[2] with brake pressure,
    //   // so this version avoids clobbering the speed bytes.
    //   uint16_t speed_m_s = 10;  // 36 km/h
    //   uint16_t speed_value = (uint16_t)(speed_m_s * 63.72f);
    //   payload[2] = speed_value & 0xFF;
    //   payload[3] = (speed_value >> 8) & 0xFF;

    //   float brake_pressure_kPa = 1024.0f;
    //   uint16_t brake_raw = (uint16_t)(brake_pressure_kPa / 128.0f);
    //   payload[0] = brake_raw & 0xFF;
    //   payload[1] = (brake_raw >> 8) & 0xFF;
    //   break;
    // }

    case 0x0D2: {
      // Placeholder
      payload[0] = 0x11;
      payload[1] = 0x22;
      payload[2] = 0x33;
      payload[3] = 0x44;
      break;
    }

    case 0x0D3: {
      // Placeholder
      payload[0] = 0x55;
      payload[1] = 0x66;
      payload[2] = 0x77;
      payload[3] = 0x88;
      break;
    }

    // case 0x0D4: {
    //   // Wheel speed sensors / ABS
    //   uint16_t value = (uint16_t)(10 * 61);
    //   payload[0] = value & 0xFF;
    //   payload[1] = (value >> 8) & 0xFF;

    //   value = (uint16_t)(10 * 62);
    //   payload[2] = value & 0xFF;
    //   payload[3] = (value >> 8) & 0xFF;

    //   value = (uint16_t)(10 * 63);
    //   payload[4] = value & 0xFF;
    //   payload[5] = (value >> 8) & 0xFF;

    //   value = (uint16_t)(10 * 64);
    //   payload[6] = value & 0xFF;
    //   payload[7] = (value >> 8) & 0xFF;
    //   break;
    // }

    // case 0x140: {
    //   uint8_t accelerator_pedal_percent = 42;
    //   payload[0] = accelerator_pedal_percent * 255 / 100;

    //   bool clutch_down = false;
    //   payload[1] = clutch_down ? 0x80 : 0x00;

    //   uint16_t rpm = 3456;
    //   payload[2] = rpm & 0xFF;
    //   payload[3] = (rpm >> 8) & 0x3F;
    //   break;
    // }

    case 0x141: {
      // Placeholder
      payload[0] = 0x01;
      payload[1] = 0x02;
      payload[2] = 0x03;
      payload[3] = 0x04;
      break;
    }

    case 0x142: {
      // Placeholder
      payload[0] = 0x10;
      payload[1] = 0x20;
      payload[2] = 0x30;
      payload[3] = 0x40;
      break;
    }

    case 0x144: {
      // Placeholder
      payload[0] = 0xAA;
      payload[1] = 0xBB;
      payload[2] = 0xCC;
      payload[3] = 0xDD;
      break;
    }

    case 0x152: {
      // Placeholder
      payload[0] = 0x12;
      payload[1] = 0x34;
      payload[2] = 0x56;
      payload[3] = 0x78;
      break;
    }

    case 0x156: {
      // Placeholder
      payload[0] = 0x9A;
      payload[1] = 0xBC;
      payload[2] = 0xDE;
      payload[3] = 0xF0;
      break;
    }

    case 0x280: {
      // Placeholder
      payload[0] = 0x01;
      break;
    }

    case 0x282: {
      // Placeholder
      payload[0] = 0x02;
      break;
    }

    case 0x284: {
      // Placeholder
      payload[0] = 0x03;
      break;
    }

    case 0x360: {
      uint8_t oil_temp_c = 100;
      uint8_t coolant_temp_c = 90;
      payload[2] = oil_temp_c + 40;
      payload[3] = coolant_temp_c + 40;
      break;
    }

    default:
      break;
  }
}

// -------------------------------------------------------------------------------------------------
// Setup / loop
// -------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("Initializing CAN1 (TWAI) on ESP32-CAN-X2...");

  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN1_TX, (gpio_num_t)CAN1_RX, TWAI_MODE_NO_ACK);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    Serial.printf("twai_driver_install failed: %d\n", err);
    while (true) delay(1000);
  }

  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("twai_start failed: %d\n", err);
    while (true) delay(1000);
  }

  Serial.println("CAN1 started at 500 kbit/s");
}

void loop() {
  uint32_t nowUs = micros();

  for (size_t i = 0; i < sizeof(scheduledPids) / sizeof(scheduledPids[0]); i++) {
    ScheduledPid &entry = scheduledPids[i];
    const uint32_t periodUs = hzToPeriodUs(entry.rate_hz);

    if ((uint32_t)(nowUs - entry.last_sent_us) >= periodUs) {
      uint8_t payload[8];
      generatePayload(entry.pid, payload);

      if (sendFrame(entry.pid, payload, 8)) {
        entry.last_sent_us = nowUs;
      } else {
        // Don't spam this too hard on a busy bus
        Serial.printf("TX failed for 0x%03X\n", entry.pid);
      }
    }
  }

  // Keep loop tight but not stupidly busy
  delayMicroseconds(200);
}