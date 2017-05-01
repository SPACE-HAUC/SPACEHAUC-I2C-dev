/*!
 * @file
 */

// Copyright 2016 - 2017 David Baumann and Jacob Hempel, UMass Lowell C&DH Team

#include "../include/spacehauc-i2c-dev.h"
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <exception>

using std::vector;
using std::string;
using std::cout;
using std::endl;
using std::stringstream;

class BUS_INIT_FAILURE : public std::exception {
 public:
    const char* what() const throw() {
    return "I2C Bus Failed To Open";
  }
} bus_init_failure;

class READ_FAILURE : public std::exception {
 public:
  const char* what() const throw() {
    return "Failed to read data from I2C bus";
  }
} read_failure;

class WRITE_FAILURE : public std::exception {
 public:
  const char* what() const throw() {
    return "Failed to write data to I2C bus";
  }
} write_failure;

class INIT_FAILURE : public std::exception {
 public:
  const char* what() const throw() {
    return "Device Failed to initialize";
  }
} init_failure;


/*!
 * converts a base 10 number into hexadecimal
 *
 * @param decimal A number in base 10 to be converted to hexadecimal in a string
 * @return a string that contains the decimal number in base 16
 */
string spacehauc_i2c::toHexString(uint8_t decimal) {
  std::stringstream stream;
  stream << "0x" <<  std::setfill ('0') << std::setw(sizeof(uint8_t)*2)
    << std::hex << static_cast<unsigned int>(decimal);
  return stream.str();
}

int spacehauc_i2c::I2C_Bus::file = -1;  // default to not opened

/*!
* initBus attempts to enable the i2c bus number that is passed to it. This bus
* can now be written to and read from.
*
* @param bus The i2c bus number to be enabled. (Almost always 1)
* @return success/failure
*/
void spacehauc_i2c::I2C_Bus::init(int bus) {
  string I2C_bus_name = "/dev/i2c-" + std::to_string(bus);
  file = open(I2C_bus_name.c_str(), O_RDWR);
  if (file <= 0) {
    throw bus_init_failure;
  }
}
/*!
 * Destructor for an I2C_Bus object
 */
spacehauc_i2c::I2C_Bus::~I2C_Bus() {}

/*!
 * Wrapper function for ioctl system call
 *
 * @param packets pointer to a system defined structure that contains messages
 * @return number of messages exchanged
 */
int spacehauc_i2c::I2C_Bus::I2C_ctl(i2c_rdwr_ioctl_data *packets) {
  return ioctl(file, I2C_RDWR, packets);
}

/*!
 * Default Constructor for I2C Device. Currently does nothing.
 */
spacehauc_i2c::I2C_Device::I2C_Device() {}

/*!
 * Destructor for I2C Device. Currently has no functionality.
 */
spacehauc_i2c::I2C_Device::~I2C_Device() {}

/*!
 * readBytes first edits the i2c_rdwr_ioctl_data and i2c_msg structs to include
 * parameter data. It then calls the ioctl() function to read data from data
 * register. This data is stored in the buffer
 *
 * @param reg The data register to read from.
 * @param *buffer Pointer to a buffer to store read data.
 * @param count The number of elements in the buffer.
 *
 * @return number of bytes exchanged
 */
int spacehauc_i2c::I2C_Device::readBytes(uint8_t reg, uint8_t *buffer,
    uint8_t count) {
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

  int returnVal = I2C_ctl(&packets);
  if (returnVal <= 0) {
    throw read_failure;
  }
  return returnVal;
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
 * @return number of bytes exchanged
 */
int spacehauc_i2c::I2C_Device::writeBytes(uint8_t reg, uint8_t buffer[],
    uint8_t count) {
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

  int returnVal = I2C_ctl(&packets);
  if (returnVal <= 0) {
    throw write_failure;
  }
  return returnVal;
}

string spacehauc_i2c::I2C_Device::getName() {
  return deviceName;
}

/*!
 * Constructor for a LuminositySensor object.
 *
 * @param address This is the address of the sensor on the bus. See sensor
 *        datasheet for info.
 */
spacehauc_i2c::TSL2561::TSL2561(uint8_t address) {
  mAddress = address;
  deviceName = "TSL2561_" + toHexString(address);
}

/*!
 * Destructor for a TSL2561 object.
 */
spacehauc_i2c::TSL2561::~TSL2561() {}

/*!
 * init() initializes the Luminosity Sensor by writing 0x03 to
 * the first control register (turning it on) and writing 0x02 to the second
 * one, enabling the sensor to read data.
 *
 * @return success/failure
 */
void spacehauc_i2c::TSL2561::init() {
  // Select control register(0x00 | 0x80)
  // Power ON mode(0x03)
  uint8_t data = 0x03;
  try {
    writeBytes(controlRegister1 | 0x80, &data, 1);
  } catch (...) {
    throw init_failure;
  }
  // Select timing register(0x01 | 0x80)
  // Nominal integration time = 402ms(0x02)
  data = 0x02;
  try {
    writeBytes(controlRegister2 | 0x80, &data, 1);
  } catch (...) {
    throw init_failure;
  }
}

/*!
 * read() reads data from the Luminosity sensor
 *
 * @return The luminosity in lux if data was read, or -1 if data couldn't be.
 */
double spacehauc_i2c::TSL2561::read() {
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

/*!
 * Constructor for the Adafruit MCP9808 I2C Temperature Sensor
 * More info on the sensor can be found at https://www.adafruit.com/product/1782
 *
 *  @param address The i2c bus address that the sensor is on
 */
spacehauc_i2c::MCP9808::MCP9808(uint8_t address) {
  mAddress = address;
  deviceName = "MCP9808_" + toHexString(address);
}

/*!
 * Destructor for a MCP9808 object
 */
spacehauc_i2c::MCP9808::~MCP9808() {}

/*!
 * init Initializes the sensor by enabling the control register for continuous
 * conversion, and by setting the resolution on the resolutuion register.
 *
 * @return success/failure
 */
void spacehauc_i2c::MCP9808::init() {
  // set up control register for continuous conversion
  uint8_t ctlData[] = {0x00, 0x00};
  try {
    writeBytes(controlRegister, ctlData, 2);
  } catch (...) {
    throw init_failure;
  }
  uint8_t resolutionData[] = {0x03};  // resolution = 0.0625 Cs
  try {
    writeBytes(resolutionRegister, resolutionData, 1);
  } catch (...) {
    throw init_failure;
  }
}

/*!
 * read Reads the current temperature from the sensor and returns that value
 * in degrees Celsius.
 *
 * @return current temperature in degrees Celsius
 */
double spacehauc_i2c::MCP9808::read() {
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

/*!
 * default constructor for LSM303 accelerometer
 */
spacehauc_i2c::LSM303_Accelerometer::LSM303_Accelerometer(uint8_t address) {
  mAddress = address;
  deviceName = "LSM303_" + toHexString(address);
}

spacehauc_i2c::LSM303_Accelerometer::~LSM303_Accelerometer() {}

/*!
 * initializes the accelerometer on LSM303 chip
 * sets data rate to 100Hz, regular power mode, no high pass filter, +- 2G scale
 */
void spacehauc_i2c::LSM303_Accelerometer::init() {
  uint8_t powerData[] = {0x57};  // enable accelerometer for 100 hz data rate,
  // regular power operation
  try {
    writeBytes(controlRegister1, powerData, 1);
  } catch (...) {
    throw init_failure;
  }

  uint8_t filterData[] = {0x00};  // no high pass filter enabled
  try {
    writeBytes(controlRegister2, filterData, 1);
  } catch (...) {
    throw init_failure;
  }

  uint8_t measurementData[] = {0x00};  // continuous data, little endian,
  // +- 2G scale, high resolution disabled
  try {
    writeBytes(controlRegister4, measurementData, 1);
  } catch (...) {
    throw init_failure;
  }
}

/*!
 * reads data from Accelerometer, returns vector sum of X, Y, and Z as double
 */
double spacehauc_i2c::LSM303_Accelerometer::read() {
  uint8_t buffer[6] = {0};
  Triplet<double> accData;
  double magnitude = 0;
  readBytes(dataRegister1, &(buffer[0]), 1);
  readBytes(dataRegister2, &(buffer[1]), 1);
  readBytes(dataRegister3, &(buffer[2]), 1);
  readBytes(dataRegister4, &(buffer[3]), 1);
  readBytes(dataRegister5, &(buffer[4]), 1);
  readBytes(dataRegister6, &(buffer[5]), 1);
  accData.setX(static_cast<int16_t>((buffer[1] << 8) + buffer[0]));
  accData.setX((accData.getX() / 32768.0) * 2);  // divide by total data points
  // (2^15) then multiply by scale (2)
  accData.setY(static_cast<int16_t>((buffer[3] << 8) + buffer[2]));
  accData.setY((accData.getY() / 32768.0) * 2);
  accData.setZ(static_cast<int16_t>((buffer[5] << 8) + buffer[4]));
  accData.setZ((accData.getZ() / 32768.0) * 2 * -1);  // negate to make down neg

  magnitude = (pow(accData.getX(), 2) + (pow(accData.getY(), 2))+
              (pow(accData.getZ(), 2)));
  return sqrt(magnitude);
}

/*!
 * reads data from accelerometer, returns a Triplet of X, Y, and Z
 */
spacehauc_i2c::Triplet<double> spacehauc_i2c::LSM303_Accelerometer::
    readTriplet() {
  uint8_t buffer[6] = {0};
  Triplet<double> accData;
  readBytes(dataRegister1, &(buffer[0]), 1);
  readBytes(dataRegister2, &(buffer[1]), 1);
  readBytes(dataRegister3, &(buffer[2]), 1);
  readBytes(dataRegister4, &(buffer[3]), 1);
  readBytes(dataRegister5, &(buffer[4]), 1);
  readBytes(dataRegister6, &(buffer[5]), 1);
  accData.setX(static_cast<int16_t>((buffer[1] << 8) + buffer[0]));
  accData.setX((accData.getX() / 32768.0) * 2);  // divide by total data points
  // (2^15) then multiply by scale (2)
  accData.setY(static_cast<int16_t>((buffer[3] << 8) + buffer[2]));
  accData.setY((accData.getY() / 32768.0) * 2);
  accData.setZ(static_cast<int16_t>((buffer[5] << 8) + buffer[4]));
  accData.setZ((accData.getZ() / 32768.0) * 2 * -1);  // negate to make down neg
  return accData;
}

/*!
 *  constructor for magnetometer, default address 0x1E
 */
spacehauc_i2c::LSM303_Magnetometer::LSM303_Magnetometer(uint8_t address) {
  mAddress = address;
  deviceName = "LSM303_" + toHexString(address);
}

spacehauc_i2c::LSM303_Magnetometer::~LSM303_Magnetometer() {}

/*!
 * initializes magnetometer, disables thermometer
 * data rate = 30 Hz, +- 1.3 Gauss, continuous conversion
 */
void spacehauc_i2c::LSM303_Magnetometer::init() {
  uint8_t setupData[] = {0x14};  // disable thermometer, data rate min = 30 Hz
  try {
    writeBytes(controlRegister1, setupData, 1);
  } catch (...) {
    throw init_failure;
  }

  uint8_t gainData[] = {0x20};  // set gain to +-1.3 Gauss
  try {
    writeBytes(controlRegister2, gainData, 1);
  } catch (...) {
    throw init_failure;
  }

  uint8_t measurementData[] = {0x00};  // continuous conversion
  try {
    writeBytes(controlRegister3, measurementData, 1);
  } catch (...) {
    throw init_failure;
  }
}

/*!
 * reads data from Magnetometer, returns vector sum of X, Y, and Z as double
 */
double spacehauc_i2c::LSM303_Magnetometer::read() {
  uint8_t buffer[6] = {0};
  Triplet<double> magData;
  double magnitude = 0;
  readBytes(dataRegister1, &(buffer[0]), 1);
  readBytes(dataRegister2, &(buffer[1]), 1);
  readBytes(dataRegister3, &(buffer[2]), 1);
  readBytes(dataRegister4, &(buffer[3]), 1);
  readBytes(dataRegister5, &(buffer[4]), 1);
  readBytes(dataRegister6, &(buffer[5]), 1);

  magData.setX(static_cast<int16_t>((buffer[1] << 8) + buffer[0]));
  magData.setX((magData.getX() / 32768.0) * 1.3);  // divide by num data points
  // (2^15) then multiply by scale (1.3)
  magData.setY(static_cast<int16_t>((buffer[3] << 8) + buffer[2]));
  magData.setY((magData.getY() / 32768.0) * 1.3);
  magData.setZ(static_cast<int16_t>((buffer[5] << 8) + buffer[4]));
  magData.setZ((magData.getZ() / 32768.0) * 1.3);

  magnitude = (pow(magData.getX(), 2) + (pow(magData.getY(), 2))+
              (pow(magData.getZ(), 2)));
  return sqrt(magnitude);
}
/*!
 * reads data from Magnetometer, Triplet of X, Y, and Z as doubles
 */
spacehauc_i2c::Triplet<double> spacehauc_i2c::LSM303_Magnetometer::
    readTriplet() {
  uint8_t buffer[6] = {0};
  Triplet<double> magData;
  readBytes(dataRegister1, &(buffer[0]), 1);
  readBytes(dataRegister2, &(buffer[1]), 1);
  readBytes(dataRegister3, &(buffer[2]), 1);
  readBytes(dataRegister4, &(buffer[3]), 1);
  readBytes(dataRegister5, &(buffer[4]), 1);
  readBytes(dataRegister6, &(buffer[5]), 1);

  magData.setX(static_cast<int16_t>((buffer[1] << 8) + buffer[0]));
  magData.setX((magData.getX() / 32768.0) * 1.3);  // divide by num data points
  // (2^15) then multiply by scale (1.3)
  magData.setY(static_cast<int16_t>((buffer[3] << 8) + buffer[2]));
  magData.setY((magData.getY() / 32768.0) * 1.3);
  magData.setZ(static_cast<int16_t>((buffer[5] << 8) + buffer[4]));
  magData.setZ((magData.getZ() / 32768.0) * 1.3);
  return magData;
}


spacehauc_i2c::L3GD20::L3GD20(uint8_t address) {
  mAddress = address;
  deviceName = "L3GD20_" + toHexString(address);
}

spacehauc_i2c::L3GD20::~L3GD20() {}

void spacehauc_i2c::L3GD20::init() {
  uint8_t setupData[] = {0x0F};  // power on, data rate 95Hz, cut-off 12.5,
  // xyz enabled
  try {
    writeBytes(controlRegister1, setupData, 1);
  } catch (...) {
    throw init_failure;
  }

  uint8_t filterData[] = {0x00};  // disable high pass filtering
  try {
    writeBytes(controlRegister2, filterData, 1);
  } catch (...) {
    throw init_failure;
  }

  uint8_t measurementData[] = {0x00};  // continuous conversion, little endian,
  // 250 degrees/second
  try {
    writeBytes(controlRegister4, measurementData, 1);
  } catch (...) {
    throw init_failure;
  }
}

double spacehauc_i2c::L3GD20::read() {
  uint8_t buffer[6] = {0};
  Triplet<double> gyroData;
  double magnitude;
  readBytes(dataRegister1, &(buffer[0]), 1);
  readBytes(dataRegister2, &(buffer[1]), 1);
  readBytes(dataRegister3, &(buffer[2]), 1);
  readBytes(dataRegister4, &(buffer[3]), 1);
  readBytes(dataRegister5, &(buffer[4]), 1);
  readBytes(dataRegister6, &(buffer[5]), 1);

  gyroData.setX(static_cast<int16_t>((buffer[1] << 8) + buffer[0]));
  gyroData.setX((gyroData.getX() / 32768.0) * 250);  // divide by data points
  // (2^15) then multiply by scale (250)
  gyroData.setY(static_cast<int16_t>((buffer[3] << 8) + buffer[2]));
  gyroData.setY((gyroData.getY() / 32768.0) * 250);
  gyroData.setZ(static_cast<int16_t>((buffer[5] << 8) + buffer[4]));
  gyroData.setZ((gyroData.getZ() / 32768.0) * 250);
  magnitude = (pow(gyroData.getX(), 2) + (pow(gyroData.getY(), 2))+
              (pow(gyroData.getZ(), 2)));
  return sqrt(magnitude);
}

spacehauc_i2c::Triplet<double> spacehauc_i2c::L3GD20::readTriplet() {
  uint8_t buffer[6] = {0};
  Triplet<double> gyroData;
  readBytes(dataRegister1, &(buffer[0]), 1);
  readBytes(dataRegister2, &(buffer[1]), 1);
  readBytes(dataRegister3, &(buffer[2]), 1);
  readBytes(dataRegister4, &(buffer[3]), 1);
  readBytes(dataRegister5, &(buffer[4]), 1);
  readBytes(dataRegister6, &(buffer[5]), 1);

  gyroData.setX(static_cast<int16_t>((buffer[1] << 8) + buffer[0]));
  gyroData.setX((gyroData.getX() / 32768.0) * 250);  // divide by data points
  // (2^15) then multiply by scale (250)
  gyroData.setY(static_cast<int16_t>((buffer[3] << 8) + buffer[2]));
  gyroData.setY((gyroData.getY() / 32768.0) * 250);
  gyroData.setZ(static_cast<int16_t>((buffer[5] << 8) + buffer[4]));
  gyroData.setZ((gyroData.getZ() / 32768.0) * 250);
  return gyroData;
}

// centers string in width w
std::string center(const string s, const int w) {
    std::stringstream ss, spaces;
    int padding = w - s.size();                 // count excess room to pad
    for (int i = 0; i < padding / 2; ++i)
        spaces << " ";
    ss << spaces.str() << s << spaces.str();    // format with padding
    if ( padding > 0 && padding % 2 != 0)         // if odd #, add 1 space
        ss << " ";
    return ss.str();
}

// sets number of decmal digits and right aligns double in width
std::string prd(const double x, const int decDigits, const int width) {
    std::stringstream ss;
    ss << std::fixed << std::right;
    ss.fill(' ');        // fill space around displayed #
    ss.width(width);     // set  width around displayed #
    ss.precision(decDigits);  // set # places after decimal
    ss << x;
    return ss.str();
}

// prints a formatted table of measured values from each sensor in 9Dof
void spacehauc_i2c::Adafruit9DOF::printTable(int numRows, int uSecDelay) {
  string name;
  Triplet<double> data;
  for (int i = 0; i < numRows; i++) {
    if (i % 10 == 0) {
      // header
      name = "Accelerometer ";
      name += accelerometer.getName();
      cout << "|" << center(name, 32) << "|";
      name = "Magnetometer ";
      name += magnetometer.getName();
      cout << center(name, 32) << "|";
      name = "GyroScope ";
      name += gyroscope.getName();
      cout << center(name, 32) << "|" << endl;

      for (int i = 0; i < 100; i++) cout << "-";
      cout << endl;

      cout << "|" << center("X", 10) << "|" << center("Y", 10) << "|"
      << center("Z", 10) << "|" << center("X", 10) << "|" << center("Y", 10)
      << "|" << center("Z", 10) << "|" << center("X", 10) << "|"
      << center("Y", 10) << "|" << center("Z", 10) << "|" << endl;
      for (int i = 0; i < 100; i++) cout << "-";
      cout << endl;
    }
    // data
    data = accelerometer.readTriplet();
    cout << "|" << center(prd(data.getX(), 3, 5), 10) << "|"
         << center(prd(data.getY(), 3, 5), 10) << "|"
         << center(prd(data.getZ(), 3, 5), 10);
    data = magnetometer.readTriplet();
    cout << "|" << center(prd(data.getX(), 3, 5), 10) << "|"
         << center(prd(data.getY(), 3, 5), 10) << "|"
         << center(prd(data.getZ(), 3, 5), 10);
    data = gyroscope.readTriplet();
    cout << "|" << center(prd(data.getX(), 3, 5), 10) << "|"
         << center(prd(data.getY(), 3, 5), 10) << "|"
         << center(prd(data.getZ(), 3, 5), 10) << "|" << endl;
    for (int j = 0; j < 100; j++) cout << "-";
    cout << endl;
    usleep(uSecDelay);
  }
}
