/*!
 * @file
 */

// Copyright 2016 - 2017 David Baumann and Jacob Hempel, UMass Lowell C&DH Team



#ifndef INCLUDE_SPACEHAUC_I2C_MOCK_H_
#define INCLUDE_SPACEHAUC_I2C_MOCK_H_

#include "spacehauc-i2c-dev.h"

namespace spacehauc_i2c_mock {

  class I2C_Bus : public spacehauc_i2c::I2C_Bus {
  public:
    static bool init(int busNum);
  };

  class TSL2561 : public spacehauc_i2c::TSL2561 {
  public:
    TSL2561(uint8_t address) : spacehauc_i2c::TSL2561(address) {}

    bool init();
    double read();
  };

  class MCP9808 : public spacehauc_i2c::MCP9808 {
  public:
    MCP9808(uint8_t address) : spacehauc_i2c::MCP9808(address) {}

    bool init();
    double read();
  };
}  // namespace spacehauc_i2c_mock

#endif  // INCLUDE_SPACEHAUC_I2C_MOCK_H_
