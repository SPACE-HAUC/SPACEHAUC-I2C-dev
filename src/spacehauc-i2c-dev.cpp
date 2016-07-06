/*!
 * @file
 */

// Copyright 2016 UMass Lowell Command and Data Handling Team

#include "../include/spacehauc-i2c-dev.h"
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string>
#include <vector>

using std::string;

/*!
 * initBus attempts to enable the i2c bus number that is passed to it. This bus
 * can now be written to and read from.
 *
 * @param bus The i2c bus number to be enabled. (Almost always 1)
 * @return success/failure
 */
bool initBus(int bus, int *file) {
  string I2C_device_name = "/dev/i2c-" + std::to_string(bus);
  *file = open(I2C_device_name.c_str(), O_RDWR);
  if (*file) {
    return true;
  }
  return false;
}

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
 * @param reg The register to write to.
 * @param *buffer A pointer to an array of input data.
 * @param count The number of elements in the input.
 *
 * @return success/failure
 */
int I2C_Device::writeBytes(uint8_t reg, uint8_t *buffer, uint8_t count) {
  vector<uint8_t> temp;
  input.push_back(reg);
  for (int i = 0; i < count; ++i) {
    input.push_back(buffer[i]);
  }
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[1];

  messages[0].addr  = mAddress[0];
  messages[0].flags = 0;
  messages[0].len   = count;
  messages[0].buf   = &input[0];

  packets.msgs      = messages;
  packets.nmsgs     = 1;

  return ioctl(mFile, I2C_RDWR, &packets) >= 0;
}

/*!
 * Constructor for a Temperature Sensor. It initializes member variables based
 * on input parameters
 *
 * @param file This is the i2c bus file that the sensor is connected to.
 * @param address This is the address of the sensor on the bus. See sensor
 *        datasheet for info.
 * @param ID_register This is the register that identifies the device. See
 *        sensor datasheet for info.
 * @param controlRegister This is the register that controls the temperature
 *        sensor. See sensor datasheet for info.
 * @param dataRegister This is the register that measured temperature data is
 *        stored in. See sensor datasheet for info.
 */
TemperatureSensor::TemperatureSensor(int file, uint8_t address,
  uint8_t ID_register, uint8_t controlRegister, uint8_t dataRegister) {
  mFile = file;
  mAddress.push_back(address);
  mID_Regsiters.push_back(ID_register);
  mControlRegisters.push_back(controlRegister);
  mDataRegisters.push_back(dataRegister);
}

/*!
 * Destructor for a Temperature Sensor.
 */
TemperatureSensor::~TemperatureSensor() {}

/*!
 * initTempSensor initializes the temperature sensor by writing to the control
 * register.
 *
 * @return  success/failure
 */
bool TemperatureSensor::initTempSensor() {
  uint8_t data = 0x98;
  if (!writeBytes(mControlRegisters[0], &data, 2)) {
    return false;
  }
  return true;
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
 * @param file This is the i2c bus file that the sensor is connected to.
 * @param address This is the address of the sensor on the bus. See sensor
 *        datasheet for info.
 * @param ID_register This is the register that identifies the device. See
 *        sensor datasheet for info.
 * @param controlRegister1 This is the first register that controls the
 *        sensor. See sensor datasheet for info.
 * @param controlRegister2 This is the second register that controls the
 *        sensor. See sensor datasheet for info.
 * @param dataRegister This is the register that measured magnetic data is
 *        stored in. See sensor datasheet for info.
 */
Magnetometer::Magnetometer(int file, uint8_t address, uint8_t ID_register,
    uint8_t controlRegister1, uint8_t controlRegister2, uint8_t dataRegister,
    MagScale scale) {
  mFile = file;
  mAddress.push_back(address);
  mID_Regsiters.push_back(ID_register);
  mControlRegisters.push_back(controlRegister1);
  mControlRegisters.push_back(controlRegister2);
  mDataRegisters.push_back(dataRegister);
  mScale = scale;
}

/*!
 * Destructor for a Magnetometer.
 */
Magnetometer::~Magnetometer() {}

/*!
 * initMagnetometer() initializes the Magnetometer by enabling the registers
 *
 * @return success/failure
 */
bool Magnetometer::initMagnetometer() {
  uint8_t scale = (uint8_t) mScale << 5;
  // all other bits 0
  uint8_t data = 0x00;
  if (!writeBytes(mControlRegisters[1], &data, 2)) {
    return false;
  }
  // continuous conversion mode
  if (!writeBytes(mControlRegisters[1], &scale, 2)) {
    return false;
  }
  return true;
}

/*!
 * readMagnetometer() reads data from the magnetometer's data register and
 * converts it into floats.
 *
 * @return data The struct of x, y, and z components of the magnetic field.
 */
fTriplet Magnetometer::readMagnetometer() {
  fTriplet data = {0};
  uint8_t buffer[6] = {0};
  readBytes(mDataRegisters[0], buffer, 6);
  data.x = ((buffer[1] << 8) | buffer[0]);
  data.y = ((buffer[3] << 8) | buffer[2]);
  data.z = ((buffer[3] << 8) | buffer[4]);
  data.x = (data.x)/*- m_bias.x) * m_scale.x */* mMagScaleValue[mScale];
  data.y = (data.y)/*- m_bias.y) * m_scale.y */* mMagScaleValue[mScale];
  /* invert z axis so it's positive down like other sensors */
  data.z = -(data.z)/*- m_bias.z) * m_scale.z */* mMagScaleValue[mScale];
  return data;
}

/*!
 * Constructor for a LuminositySensor object.
 *
 * @param file This is the i2c bus file that the sensor is connected to.
 * @param address This is the address of the sensor on the bus. See sensor
 *        datasheet for info.
 * @param ID_register This is the register that identifies the device. See
 *        sensor datasheet for info.
 * @param controlRegister1 This is the first register that controls the
 *        sensor. See sensor datasheet for info.
 * @param controlRegister2 This is the second register that controls the
 *        sensor. See sensor datasheet for info.
 * @param dataRegister This is the register that measured magnetic data is
 *        stored in. See sensor datasheet for info.
 */
LuminositySensor::LuminositySensor(int file, uint8_t address,
    uint8_t ID_register, uint8_t controlRegister1, uint8_t controlRegister2,
    uint8_t dataRegister) {
  mFile = file;
  mAddress.push_back(address);
  mID_Regsiters.push_back(ID_register);
  mControlRegisters.push_back(controlRegister1);
  mControlRegisters.push_back(controlRegister2);
  mDataRegisters.push_back(dataRegister);
}

/*!
 * Destructor for a LuminositySensor object.
 */
LuminositySensor::~LuminositySensor() {}

/*!
 * initLuminositySensor() initializes the Luminosity Sensor by writing 0x03 to
 * the first control register (turning it on) and writing 0x02 to the second
 * one, enabling the sensor to read data.
 *
 * @return success/failure
 */
bool LuminositySensor::initLuminositySensor() {
  // Select control register(0x00 | 0x80)
  // Power ON mode(0x03)
  uint8_t *data;
  *data = 0x03
  if (!(writeBytes(mControlRegisters[0] | 0x80, data, 2))) {
    return false;
  }
  // Select timing register(0x01 | 0x80)
  // Nominal integration time = 402ms(0x02)
  *data = 0x02;
  if (!(writeBytes(mControlRegisters[1] | 0x80, data, 2))) {
    return false;
  }
  return true;
}

/*!
 * readLuminositySensor() reads data from the Luminosity sensor
 *
 * @return The luminosity in lux if data was read, or -1 if data couldn't be.
 */
double LuminositySensor::readLuminositySensor() {
  // Read 4 bytes of data from register(0x0C | 0x80)
  // ch0 lsb, ch0 msb, ch1 lsb, ch1 msb
  uint8_t input = 0;
  if (!writeBytes(mDataRegisters[0] | 0x80, &input, 1)) {
    return -1;  // error
  }
  uint8_t data[2] = {0};
  if (!readBytes(mDataRegisters[0], data, 2)) {
    return -1;  // error
  }
  /*if (!readBytes(0x0D, &data[1], 1)) {
    return -1;  // error
  }*/
  // Convert the data (if 2 bytes read)
  return ((data[1]) * 256 + data[0]);
}
