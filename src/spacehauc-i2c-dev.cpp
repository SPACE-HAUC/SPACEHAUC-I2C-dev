/*!
 * @file
 */

// Copyright 2016 UMass Lowell Command and Data Handling Team

#include "../include/spacehauc-i2c-dev.h"
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>

using std::string;
using std::cout;
using std::endl;
using std::stringstream;

using spacehauc_i2c::I2C_Bus;
using spacehauc_i2c::I2C_Device;
using spacehauc_i2c::MCP9808;
using spacehauc_i2c::TSL2561;

/*!
 * converts a 8 bit number into hexadecimal
 */

string spacehauc_i2c::toHexString(uint8_t decimal) {
  std::stringstream stream;
  stream << "0x" <<  std::setfill ('0') << std::setw(sizeof(uint8_t)*2)
  << std::hex << static_cast<unsigned int>(decimal);
  return stream.str();
}

int I2C_Bus::file = -1;  // default to not opened

/*!
* initBus attempts to enable the i2c bus number that is passed to it. This bus
* can now be written to and read from.
*
* @param bus The i2c bus number to be enabled. (Almost always 1)
* @return success/failure
*/
bool I2C_Bus::init(int bus) {
  string I2C_bus_name = "/dev/i2c-" + std::to_string(bus);
  file = open(I2C_bus_name.c_str(), O_RDWR);
  if (file > 0) {
    return true;
  }
  return false;
}


I2C_Bus::~I2C_Bus() {}

int I2C_Bus::I2C_ctl(i2c_rdwr_ioctl_data *packets) {
  return ioctl(file, I2C_RDWR, packets);
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
  messages[0].addr  = mAddress;
  messages[0].flags = 0;
  messages[0].len   = 1;
  messages[0].buf   = &reg;

  /* read */
  messages[1].addr  = mAddress;
  messages[1].flags = I2C_M_RD;
  messages[1].len   = count;
  messages[1].buf   = buffer;

  packets.msgs      = messages;
  packets.nmsgs     = 2;

  return I2C_ctl(&packets);
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
  vector<uint8_t> input;
  input.push_back(reg);
  for (int i = 0; i < count; ++i) {
    input.push_back(buffer[i]);
  }
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[1];

  messages[0].addr  = mAddress;
  messages[0].flags = 0;
  messages[0].len   = count + 1;
  messages[0].buf   = &input[0];

  packets.msgs      = messages;
  packets.nmsgs     = 1;

  return I2C_ctl(&packets);
}

string I2C_Device::getName() {
  return deviceName;
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
TSL2561::TSL2561(uint8_t address) {
  mAddress = address;
  deviceName = "TSL2561_" + toHexString(address);
}

/*!
 * Destructor for a TSL2561 object.
 */
TSL2561::~TSL2561() {}

/*!
 * initLuminositySensor() initializes the Luminosity Sensor by writing 0x03 to
 * the first control register (turning it on) and writing 0x02 to the second
 * one, enabling the sensor to read data.
 *
 * @return success/failure
 */
bool TSL2561::init() {
  // Select control register(0x00 | 0x80)
  // Power ON mode(0x03)
  uint8_t data = 0x03;
  if (writeBytes(controlRegister1 | 0x80, &data, 1) <= 0) {
    return false;
  }
  // Select timing register(0x01 | 0x80)
  // Nominal integration time = 402ms(0x02)
  data = 0x02;
  if (writeBytes(controlRegister2 | 0x80, &data, 1) <= 0) {
    return false;
  }
  return true;
}

/*!
 * readLuminositySensor() reads data from the Luminosity sensor
 *
 * @return The luminosity in lux if data was read, or -1 if data couldn't be.
 */
double TSL2561::read() {
  // Read 4 bytes of data from register(0x0C | 0x80)
  // ch0 lsb, ch0 msb, ch1 lsb, ch1 msb
  uint8_t input = 0;
  if (writeBytes(dataRegister | 0x80, &input, 1) <= 0) {
    return -1;  // error
  }
  uint8_t data[2] = {0};
  if (readBytes(dataRegister, data, 2) <= 0) {
    return -1;  // error
  }
  // Convert the data
  return ((data[1]) * 256 + data[0]);
}


// Adafruit MCP9808 I2C Temperature Sensor
// More info https://www.adafruit.com/product/1782

MCP9808::MCP9808(uint8_t address) {
  mAddress = address;
  deviceName = "MCP9808_" + toHexString(address);
}

MCP9808::~MCP9808() {}

bool MCP9808::init() {
  // set up control register for continuous conversion
  uint8_t ctlData[] = {0x00, 0x00};
  if (writeBytes(controlRegister, ctlData, 2) <= 0) {
    return false;
  }
  uint8_t resolutionData[] = {0x03};
  if (writeBytes(resolutionRegister, resolutionData, 1) <= 0) {
    return false;
  }
  return true;
}

double MCP9808::read() {
  uint8_t data[2] = {0};
  if (readBytes(dataRegister, data, 2) <= 0) {
    return -100000;
    // not a temperature, use for error checking
  }
  // clear first 3 bits and combines
  int temp = (data[0] & 0x1F) * 256 + data[1];
  if (temp > 4095) {
    temp -= 8192;
  }
  double cTemp = temp * 0.0625;
  return cTemp;
}
