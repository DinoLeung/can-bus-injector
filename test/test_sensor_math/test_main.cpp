#include <Arduino.h>
#include <unity.h>
#include "sensor_math.h"

void test_0_ADC() {
    float zeroV = adcToVoltage(0);
    TEST_ASSERT_TRUE(isfinite(zeroV));
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, zeroV);
}

void test_3_point_3_ADC() {
    float VREF = adcToVoltage(4095);
    TEST_ASSERT_TRUE(isfinite(VREF));
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 3.3f, VREF);
}

void test_pressure_valid_3_point_7_bar() {
    constexpr float EXPECTED_BAR = 3.7f;
    constexpr float EXPECTED_PSI = EXPECTED_BAR * 14.5038f;
    // Output from sensor 400 mV/bar where 0 bar is 0.5V
    constexpr float SENSOR_V = 0.5f + EXPECTED_BAR * 0.4f;
    // Voltage divider resistors: Rtop = 5.6 kΩ, Rbot = 10 kΩ
    constexpr float DIVIDER_RATIO = 10.0f / (5.6f + 10.0f);
    float vAdc = SENSOR_V * DIVIDER_RATIO;
    float psi = computePressurePsi(vAdc);
    TEST_ASSERT_TRUE(isfinite(psi));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, EXPECTED_PSI, psi);
}

void test_pressure_invalid_voltage() {
    TEST_ASSERT_TRUE(isnan(computePressurePsi(0.0f)));
    TEST_ASSERT_TRUE(isnan(computePressurePsi(3.3f)));
}

void test_temperature_valid_90C() {
    constexpr float R_TOP = 6800.0f;
    float r = 244.0f; // resistance at 90 °C
    float vAdc = 3.3f * r / (R_TOP + r);
    float tempC = computeTemperatureC(vAdc);
    TEST_ASSERT_TRUE(isfinite(tempC));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 90.0f, tempC);
}

void test_temperature_invalid_voltage() {
    TEST_ASSERT_TRUE(isnan(computeTemperatureC(0.0f)));
    TEST_ASSERT_TRUE(isnan(computeTemperatureC(3.3f)));
}

void setup() {
    delay(2000); // for serial monitor
    UNITY_BEGIN();
    RUN_TEST(test_0_ADC);
    RUN_TEST(test_3_point_3_ADC);
    RUN_TEST(test_pressure_valid_3_point_7_bar);
    RUN_TEST(test_pressure_invalid_voltage);
    RUN_TEST(test_temperature_valid_90C);
    RUN_TEST(test_temperature_invalid_voltage);
    UNITY_END();
}

void loop() {}
