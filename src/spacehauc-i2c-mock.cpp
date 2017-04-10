#include "../include/spacehauc-i2c-mock.h"

void spacehauc_i2c_mock::I2C_Bus::init(int busNum) {
  srand(time(nullptr));
}

void spacehauc_i2c_mock::TSL2561::init() {
}

double spacehauc_i2c_mock::TSL2561::read() {
  int value = rand() % 200;
  int multiplier = rand() % 3;
  multiplier -= 1;
  value *= multiplier;
  value += 300;
  return static_cast<double>(value);
}

void spacehauc_i2c_mock::MCP9808::init() {
}

double spacehauc_i2c_mock::MCP9808::read() {
  int value = rand() % 25;
  int multiplier = rand() % 3;
  multiplier -= 1;
  value *= multiplier;
  value += 30;
  return static_cast<double>(value);
}
