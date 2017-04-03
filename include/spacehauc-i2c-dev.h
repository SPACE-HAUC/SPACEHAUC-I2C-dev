/*!
 * @file
 */

// Copyright 2016 - 2017 David Baumann and Jacob Hempel, UMass Lowell C&DH Team

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

/* converts a base 10 number to a hex value stored in a string */
string toHexString(uint8_t decimal);

class I2C_Bus {
 private:
   /*! This is an integer (file descriptor) that represents the
    * opened I2C file (device)
    */
  static int file;
 protected:
   /*! Wrapper for ioctl system call */
  int I2C_ctl(i2c_rdwr_ioctl_data *packets);
 public:
  virtual ~I2C_Bus();
  /*! initialize the bus with the given bus number */
  static bool init(int busNum);
};


/*!
 * This is a class for any i2c device to inherit from. All i2c devices need
 * these methods.
 */
class I2C_Device : public I2C_Bus {
 protected:
  /*! The address on the bus for the device*/
  uint8_t mAddress;
  /*! The ID register for the device. */
  uint8_t mID_Regsiter;
  /*! A string to hold the device's name */
  string deviceName;
  /*! read data from the bus */
  int readBytes(uint8_t reg, uint8_t *buffer, uint8_t count);
  /*! write data on the bus */
  int writeBytes(uint8_t reg, uint8_t *buffer, uint8_t count);
 public:
  I2C_Device();
  virtual ~I2C_Device();
  /*! get the device's name */
  string getName();
  /*! initialize the device */
  virtual bool init() = 0;
  /*! read data from the device */
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
 public:
  explicit MCP9808(uint8_t address);
  ~MCP9808();
  bool init();
  double read();
};
}  // namespace spacehauc_i2c

#endif  // INCLUDE_SPACEHAUC_I2C_DEV_H_
