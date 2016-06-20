/*!
 * @file
 */

// Copyright 2016 UMass Lowell Command and Data Handling Team

#include "spacehauc-i2c-dev.hpp"
#include <string>
#include <vector>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>

using std::string;

/*!
 * Constructor for I2C Device. Currently has no functionality, it may in
 * a future release.
 *
 */
I2C_Device::I2C_Device() {}

/*!
 * Destructor for I2C Device. Currently has no functionality, it may in
 * a future release.
 *
 */
I2C_Device::~I2C_Device() {}

/*!
 * initDevice sets the member variable I2C_device_name of the I2C_Device class
 * to the name of the i2c bus (usually /dev/i2c-1)
 * It then opens the bus for reading and writing.
 *
 * @return success/failure
 */
bool I2C_Device::initDevice() {
  mI2C_device_name = "/dev/i2c-" + std::to_string((int)mBus);
  mFile = open(mI2C_device_name.c_str(), O_RDWR);
  if (mFile) {
    return true;
  }
  return false;
}

/*!
 * readBytes first edits the i2c_rdwr_ioctl_data and i2c_msg structs to include
 * parameter data. It then calls the ioctl() function to read data from data
 * register. This data is stored in the buffer
 *
 * @param reg The data register to read from.
 * @param *buffer Pointer to a buffer to store read data.
 * @param count The number of elements in the buffer.
 *
 * @return success/failure
 */
int I2C_Device::readBytes(uint8_t reg, uint8_t *buffer, uint8_t count) {
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[2];
  /* write the register we want to read from */
  messages[0].addr  = mAddress[0];
  messages[0].flags = 0;
  messages[0].len   = 1;
  messages[0].buf   = &reg;

  /* read */
  messages[1].addr  = mAddress[0];
  messages[1].flags = I2C_M_RD;
  messages[1].len   = count;
  messages[1].buf   = buffer;

  packets.msgs      = messages;
  packets.nmsgs     = 2;

  return ioctl(mFile, I2C_RDWR, &packets) >= 0;
}

/*!
 * writeBytes writes input data to an i2c register. It first edits the
 * i2c_rdwr_ioctl_data and i2c_msg structs to include parameter data. Then it
 * calls the ioctl() function to write the data.
 *
 * @param count The number of elements in the input.
 * @param *input A pointer to an array of input data
 *
 * @return success/failure
 */
int I2C_Device::writeBytes(uint8_t count, uint8_t *input) {
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[1];

  messages[0].addr  = mAddress[0];
  messages[0].flags = 0;
  messages[0].len   = count;
  messages[0].buf   = input; // needs to be an array w/ first being register
    // and second being data

  packets.msgs      = messages;
  packets.nmsgs     = 1;

  return ioctl(mFile, I2C_RDWR, &packets) >= 0 ;
}

/*!
 * Constructor for a Temperature Sensor. It initializes member variables based
 * on input parameters
 *
 * @param bus This is the i2c bus that the sensor is connected to
 * @param address This is the address of the sensor on the bus. See sensor
 *        datasheet for info
 * @param ID_register This is the register that identifies the device. See
 *        sensor datasheet for info
 * @param controlRegister This is the register that controls the temperature
 *        sensor. See sensor datasheet for info.
 * @param dataRegister This is the register that measured temperature data is
 *        stored in. See sensor datasheet for info.
 */
TemperatureSensor::TemperatureSensor(uint8_t bus, uint8_t address, uint8_t ID_register,
  uint8_t controlRegister, uint8_t dataRegister) {
  mBus = bus;
  mAddress.push_back(address);
  mID_Regsiters.push_back(ID_register);
  mControlRegisters.push_back(controlRegister);
  mDataRegisters.push_back(dataRegister);
}

TemperatureSensor::~TemperatureSensor() {}

/*!
 * initTempSensor initializes the temperature sensor by writing to the control
 * register.
 *
 * @return  success/failure
 */
bool TemperatureSensor::initTempSensor() {
  uint8_t input[2] = {mControlRegisters[0], 0x98};
  if (writeBytes (2, input)) {
    return true;
  }
  return false;
}

/*!
 * readTemp reads data from the data register using readBytes()
 *
 * @return mTemperature the temperature
 */
uint8_t TemperatureSensor::readTemp() {
  readBytes(mDataRegisters[0], &mTemperature, 2);
  return mTemperature;
}

/*!
 * Constructor for a magnetometer. It initializes the member variables based on
 * input parameters.
 *
 * @param bus This is the i2c bus that the sensor is connected to
 * @param address This is the address of the sensor on the bus. See sensor
 *        datasheet for info
 * @param ID_register This is the register that identifies the device. See
 *        sensor datasheet for info
 * @param controlRegister1 This is the first register that controls the
 *        sensor. See sensor datasheet for info.
 * @param controlRegister2 This is the second register that controls the
 *        sensor. See sensor datasheet for info.
 * @param dataRegister This is the register that measured magnetic data is
 *        stored in. See sensor datasheet for info.
 */
Magnetometer::Magnetometer(uint8_t bus, uint8_t address, uint8_t ID_register,
  uint8_t controlRegister1, uint8_t controlRegister2, uint8_t dataRegister,
  MagScale scale) {
  mBus = bus;
  mAddress.push_back(address);
  mID_Regsiters.push_back(ID_register);
  mControlRegisters.push_back(controlRegister1);
  mControlRegisters.push_back(controlRegister2);
  mDataRegisters.push_back(dataRegister);
  mScale = scale;
}

Magnetometer::~Magnetometer() {};

bool Magnetometer::initMagnetometer() {
  uint8_t scale = (uint8_t) mScale << 5;
  uint8_t input2[2] = {mControlRegisters[1], 0x00};
  uint8_t input1[2] = {mControlRegisters[1], scale};
  // all other bits 0
  if (!writeBytes (2, input1)) {
    return false;
  }
  // continuous conversion mode
  if (!writeBytes (2, input2)) {
    return false;
  }
  return true;
}

fTriplet Magnetometer::readMagnetometer() {
  fTriplet data = {0};
  uint8_t buffer[6] = {0};
  readBytes(mDataRegisters[0], buffer, 6);
  data.x = ((buffer[1] << 8) | buffer[0]);
  data.y = ((buffer[3] << 8) | buffer[2]);
  data.z = ((buffer[3] << 8) | buffer[4]);
  data.x = (data.x )/*- m_bias.x) * m_scale.x */* mMagScaleValue[mScale];
  data.y = (data.y )/*- m_bias.y) * m_scale.y */* mMagScaleValue[mScale];
  /* invert z axis so it's positive down like other sensors */
  data.z = -(data.z )/*- m_bias.z) * m_scale.z */* mMagScaleValue[mScale];
  return data;
}
