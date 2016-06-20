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
 * This is a structure of integer variables x y, and z that can be easily passed
 * around by methods that measure in 3 dimensions.
 */
struct Triplet {
  int16_t x;
  int16_t y;
  int16_t z;
};

/*!
 * This is a structure of floating point variables x y, and z that can be easily passed
 * around by methods that measure in 3 dimensions.
 */
struct fTriplet {
  float x;
  float y;
  float z;
};

/*!
 * Enumerated data type for scaling the magnetometer. One of these is entered
 * into the magnetometer's constructor to set the scale.
 */
enum MagScale { MAG_SCALE_2GS, MAG_SCALE_4GS, MAG_SCALE_8GS, MAG_SCALE_12GS };

/*!
 * This is a class for any i2c device to inherit from. All i2c devices need
 * these methods.
 */
class I2C_Device {
protected:
  /*! String that holds the I2C device's file name. */
  string mI2C_device_name;
  /*! This is the bus number that we are using. */
  uint8_t mBus;
  /*! This is an integer that represents the opened I2C file (device) */
  int mFile;
  /*! A vector of each address that the device may contain. */
  vector<uint8_t> mAddress;
  /*! A vector of each of the ID registers that the device may contain. */
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
  /*! A vector of each of the data registers the temperature sensor uses. */
  vector<uint8_t> mDataRegisters;
  /*! A vector of each of the control registers the temperature sensor uses. */
  vector<uint8_t> mControlRegisters;
  /*! This is a variable to hold the measured temperature value. */
  uint8_t mTemperature;
public:
  explicit TemperatureSensor(uint8_t bus, uint8_t address, uint8_t ID_register,
    uint8_t controlRegister, uint8_t dataRegister);
  ~TemperatureSensor();
  bool initTempSensor();
  uint8_t readTemp();
};

/*!
 * This is a class for a magnetometer, specifically the 9DoF board's sensor.
 */
class Magnetometer : public I2C_Device {
private:
  /*! An array that holds each possible scaling for the magnetometer. */
  const float mMagScaleValue[5] = { 2 / 32768.0, 4 / 32768.0, 6 / 32768.0,
    8 / 32768.0, 12 / 32768.0 };
  /*! A variable of the enumerated type MagScale; is set to a scale's name.*/
  MagScale mScale;
  /*! A vector that holds the data register(s) for the magnetometer. */
  vector<uint8_t> mDataRegisters;
  /*! A vector that holds the control registers for the magnetometer. */
  vector<uint8_t> mControlRegisters;
public:
  explicit Magnetometer(uint8_t bus, uint8_t address, uint8_t ID_register,
    uint8_t controlRegister1, uint8_t controlRegister2, uint8_t dataRegister,
    MagScale scale);
  ~Magnetometer();
  bool initMagnetometer();
  fTriplet readMagnetometer();
};


#endif  // INCLUDE_SPACEHAUC_I2C_DEV_HPP_
