/*!
 * @file
 */

// Copyright 2016 - 2017 David Baumann and Jacob Hempel, UMass Lowell C&DH Team

#ifndef INCLUDE_SPACEHAUC_I2C_DEV_H_
#define INCLUDE_SPACEHAUC_I2C_DEV_H_

#include <linux/i2c-dev.h>
#include <string>
#include <iostream>

using std::string;

/*!
 * Namespace for I2C sensors/devices
 */
namespace spacehauc_i2c {

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
  static void init(int busNum);
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
  virtual void init() = 0;
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
  void init();
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
  void init();
  double read();
};

template<class T>
class Triplet {
 private:
  T X;
  T Y;
  T Z;
 public:
  Triplet() : X(0), Y(0), Z(0) {}
  Triplet(T x, T y, T z) : X(x), Y(y), Z(z) {}
  T getX() const {return X;}
  T getY() const {return Y;}
  T getZ() const {return Z;}
  void setX(T x) {X = x;}
  void setY(T y) {Y = y;}
  void setZ(T z) {Z = z;}
  Triplet<T> operator=(Triplet<T> rVal) {
    X = rVal.X;
    Y = rVal.Y;
    Z = rVal.Z;
    return rVal;
  }
};


/*!
 * This is a class for the LSM303 Accelerometer, set up to measure +-2G
 * at 100Hz
 */
class LSM303_Accelerometer : public I2C_Device {
 private:
  // const uint8_t ID_register;
  const uint8_t dataRegister1 = 0x28;  // LSB of x value
  const uint8_t dataRegister2 = 0x29;  // MSB of x value
  const uint8_t dataRegister3 = 0x2A;  // LSB of y value
  const uint8_t dataRegister4 = 0x2B;  // MSB of y value
  const uint8_t dataRegister5 = 0x2C;  // LSB of x value
  const uint8_t dataRegister6 = 0x2D;  // LSB of x value
  const uint8_t controlRegister1 = 0x20;
  const uint8_t controlRegister2 = 0x21;
  const uint8_t controlRegister4 = 0x23;
  // no need to access 3, 5, or 6; default values at those control regs is fine
 public:
  explicit LSM303_Accelerometer(uint8_t address = 0x19);
  ~LSM303_Accelerometer();
  void init();
  double read();  // reads total G force (magnitude of acceleration vector)
  Triplet<double> readTriplet();  // reads each axis of acceleration
};

/*!
 * This is a class for the LSM303 magnetometer, set up to measure +-1.3 Gauss
 * at 30Hz
 */
class LSM303_Magnetometer : public I2C_Device {
 private:
  const uint8_t dataRegister1 = 0x03;  // LSB of x value
  const uint8_t dataRegister2 = 0x04;  // MSB of x value
  const uint8_t dataRegister3 = 0x05;  // LSB of y value
  const uint8_t dataRegister4 = 0x06;  // MSB of y value
  const uint8_t dataRegister5 = 0x07;  // LSB of x value
  const uint8_t dataRegister6 = 0x08;  // LSB of x value
  const uint8_t controlRegister1 = 0x00;
  const uint8_t controlRegister2 = 0x01;
  const uint8_t controlRegister3 = 0x02;
 public:
  explicit LSM303_Magnetometer(uint8_t address = 0x1E);
  ~LSM303_Magnetometer();
  void init();
  double read();  // reads total mag field (magnitude of field vector)
  Triplet<double> readTriplet();  // reads each axis of magnetic field
};


/*!
 * THis is a class for the L3GD20 gyroscope
 */
class L3GD20 : public I2C_Device {
 private:
  const uint8_t dataRegister1 = 0x28;  // LSB of x value
  const uint8_t dataRegister2 = 0x29;  // MSB of x value
  const uint8_t dataRegister3 = 0x2A;  // LSB of y value
  const uint8_t dataRegister4 = 0x2B;  // MSB of y value
  const uint8_t dataRegister5 = 0x2C;  // LSB of x value
  const uint8_t dataRegister6 = 0x2D;  // LSB of x value
  const uint8_t controlRegister1 = 0x20;
  const uint8_t controlRegister2 = 0x21;
  const uint8_t controlRegister4 = 0x23;  // access to 3 and 5 not needed
 public:
  explicit L3GD20(uint8_t address = 0x6B);
  ~L3GD20();
  void init();
  double read();
  Triplet<double> readTriplet();
};

/*!
 * Wrapper Class for Adafruit's 9DOF board, which has 2 chips:
 * LSM303, which has accelerometer and magnetometer, and L3GD20 which has the
 * gyroscope
 */
class Adafruit9DOF {
 public:
  LSM303_Accelerometer accelerometer;
  LSM303_Magnetometer magnetometer;
  L3GD20 gyroscope;
  Adafruit9DOF() : accelerometer(0x19), magnetometer(0x1E), gyroscope(0x6B) {}
  Adafruit9DOF(uint8_t accelAddr, uint8_t magAddr, uint8_t gyroAddr) :
              accelerometer(accelAddr), magnetometer(magAddr),
              gyroscope(gyroAddr) {}
  /*! prints a formatted table of all data from each sensor */
  void printTable(int numRows = 1, int uSecDelay = 500000);
};

}  // namespace spacehauc_i2c

#endif  // INCLUDE_SPACEHAUC_I2C_DEV_H_
