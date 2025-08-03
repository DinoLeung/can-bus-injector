#pragma once

float adcToVoltage(int raw);
float computePressurePsi(float vAdc);
float computeTemperatureC(float vAdc);