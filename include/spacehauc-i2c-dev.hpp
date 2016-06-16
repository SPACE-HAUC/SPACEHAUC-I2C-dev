/*!
 * @file
 */
 
// Copyright 2016 UMass Lowell Command and Data Handling Team

#ifndef INCLUDE_SPACEHAUC_I2C_DEV_HPP_
#define INCLUDE_SPACEHAUC_I2C_DEV_HPP_

#include <string>
#include <vector>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>

using std::vector;
using std::string;

/*!
 * This is a class for any i2c device to inherit from. All i2c devices need
 * these methods.
 */
class I2C_Device {
protected:
  string mI2C_device_name;
  uint8_t mBus;
  int mFile;
  vector<uint8_t> mAddress;
  vector<uint8_t> mID_Regsiters;
  int readBytes(uint8_t reg, uint8_t *buffer, uint8_t count);
  int writeBytes(uint8_t count, uint8_t *input);
public:
  I2C_Device();
  virtual ~I2C_Device();
  bool initDevice();
};

/*!
 * This is a class for a temperature sensor, specifically the 9DoF board's
 * sensor.
 */
class TemperatureSensor : public I2C_Device {
private:
  vector<uint8_t> mDataRegisters;
  vector<uint8_t> mControlRegisters;
  uint8_t mTemperature;
public:
  explicit TemperatureSensor(uint8_t bus, uint8_t address, uint8_t ID_register,
    uint8_t controlRegister, uint8_t dataRegister);
  ~TemperatureSensor();
  bool initTempSensor();
  uint8_t readTemp();
};

#endif  // INCLUDE_SPACEHAUC_I2C_DEV_HPP_
