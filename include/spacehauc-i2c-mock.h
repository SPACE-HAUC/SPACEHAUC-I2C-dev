/*!
 * @file
 */

// Copyright 2016 - 2017 David Baumann and Jacob Hempel, UMass Lowell C&DH Team



#ifndef INCLUDE_SPACEHAUC_I2C_MOCK_H_
#define INCLUDE_SPACEHAUC_I2C_MOCK_H_

#include "./spacehauc-i2c-dev.h"
/*!
 * Namespace for mocked I2C devices
 */
namespace spacehauc_i2c_mock {
/*!
 * Overrides the I2C_Bus function init to make it always succeed for mocking purposes
 */
class I2C_Bus : public spacehauc_i2c::I2C_Bus {
 public:
  static void init(int busNum);
};

/*!
 * Overrides the class TSL2561 to make the init function always succeed and the read function
 * return a random reading within an expected range of values.
 */
class TSL2561 : public spacehauc_i2c::TSL2561 {
 public:
  explicit TSL2561(uint8_t address) : spacehauc_i2c::TSL2561(address) {}

  void init();
  double read();
};

/*!
 * Overrides the class MCP9808 to make the init function always succeed and the read function
 * return a random reading within an expected range of values.
 */
class MCP9808 : public spacehauc_i2c::MCP9808 {
 public:
  explicit MCP9808(uint8_t address) : spacehauc_i2c::MCP9808(address) {}

  void init();
  double read();
};
}  // namespace spacehauc_i2c_mock

#endif  // INCLUDE_SPACEHAUC_I2C_MOCK_H_
