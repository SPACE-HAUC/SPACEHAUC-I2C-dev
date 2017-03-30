/*!
 * @file
 */

// Copyright 2016 UMass Lowell Command and Data Handling Team

#ifndef INCLUDE_SPACEHAUC_I2C_DEV_H_
#define INCLUDE_SPACEHAUC_I2C_DEV_H_

#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string>
#include <vector>


namespace spacehauc_i2c {

using std::vector;
using std::string;

string toHexString(uint8_t decimal);

class I2C_Bus {
 private:
   /*! This is an integer (file descriptor) that represents the
    * opened I2C file (device)
    */
  static int file;
 protected:
  int I2C_ctl(i2c_rdwr_ioctl_data *packets);
 public:
  virtual ~I2C_Bus();
  static bool init(int busNum);
  static int getFile() {return file;}
};


/*!
 * This is a class for any i2c device to inherit from. All i2c devices need
 * these methods.
 */
class I2C_Device : public I2C_Bus {
 protected:
  /*! A vector of each address that the device may contain. */
  uint8_t mAddress;
  /*! A vector of each of the ID registers that the device may contain. */
  uint8_t mID_Regsiter;

  string deviceName;
  int readBytes(uint8_t reg, uint8_t *buffer, uint8_t count);
  int writeBytes(uint8_t reg, uint8_t *buffer, uint8_t count);
 public:
  I2C_Device();
  virtual ~I2C_Device();
  bool initDevice();
  string getName();
  virtual bool init() = 0;
  virtual double read() = 0;
};

/*!
 * This is a class for a luminosity sensor.
 */
class TSL2561 : public I2C_Device {
 private:
  const uint8_t ID_register = 0x0A;
  const uint8_t controlRegister1 = 0x00;
  const uint8_t controlRegister2 = 0x01;
  const uint8_t dataRegister = 0x0C;
    /*! This is a variable to hold the measured luminosity value. */
  double mLuminosity;
 public:
  explicit TSL2561(uint8_t address);
  ~TSL2561();
  bool init();
  double read();
};

/*!
 * This is a class for a temperature sensor, specifically the MCP9808
 * sensor.
 */
class MCP9808 : public I2C_Device {
 private:
  const uint8_t ID_register = 0x07;
  const uint8_t dataRegister = 0x05;
  const uint8_t controlRegister = 0x01;
  const uint8_t resolutionRegister = 0x08;
  /*! This is a variable to hold the measured temperature value. */
  uint8_t mTemperature;
 public:
  explicit MCP9808(uint8_t address);
  ~MCP9808();
  bool init();
  double read();
};
}  // namespace spacehauc_i2c
#endif  // INCLUDE_SPACEHAUC_I2C_DEV_H_
