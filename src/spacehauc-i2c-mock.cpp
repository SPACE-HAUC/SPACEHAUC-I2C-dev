/*!
 * @file
 */

// Copyright 2016 - 2017 David Baumann and Jacob Hempel, UMass Lowell C&DH Team

#include "../include/spacehauc-i2c-mock.h"

unsigned int globalSeed;

void spacehauc_i2c_mock::I2C_Bus::init(int busNum) {
  srand(time(nullptr));
  globalSeed = time(nullptr);
}

void spacehauc_i2c_mock::TSL2561::init() {
}

double spacehauc_i2c_mock::TSL2561::read() {
  int value = rand_r(&globalSeed) % 200;
  int multiplier = rand_r(&globalSeed) % 3;
  multiplier -= 1;
  value *= multiplier;
  value += 300;
  return static_cast<double>(value);
}

void spacehauc_i2c_mock::MCP9808::init() {
}

double spacehauc_i2c_mock::MCP9808::read() {
  int value = rand_r(&globalSeed) % 25;
  int multiplier = rand_r(&globalSeed) % 3;
  multiplier -= 1;
  value *= multiplier;
  value += 30;
  return static_cast<double>(value);
}
