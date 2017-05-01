// Copyright 2016 - 2017 David Baumann and Jacob Hempel, UMass Lowell C&DH Team

#include <unistd.h>
#include <iostream>
#include <string>
#include "../include/spacehauc-i2c-dev.h"
#include "../include/spacehauc-i2c-mock.h"

using std::string;
using std::cerr;
using std::cout;
using std::endl;

void testTemperatureSensor();
void testLuminositySensor();
void testMock();
void test9DoF();

// using spacehauc_i2c::I2C_Bus;
// using spacehauc_i2c::TSL2561;
// using spacehauc_i2c::MCP9808;

// using spacehauc_i2c_mock::I2C_Bus;
// using spacehauc_i2c_mock::TSL2561;
// using spacehauc_i2c_mock:: MCP9808;

int main(int argc, char* argv[]) {
  cout << "SPACEHAUC I2C Library Driver" << endl;
  cout << "------TESTING HARDWARE------" << endl;
  cout << "Initializing Bus..." << endl;
  int bus = 1;  // the first i2c bus
  try {
    spacehauc_i2c::I2C_Bus::init(bus);
  } catch (std::exception& e) {
    cout << "\nERROR: " << e.what() << "\n" << endl;
  }
  cout << "Testing Temperature Sensor..." << endl;
  try {
    testTemperatureSensor();
  } catch (std::exception& e) {
    cout << "\nERROR: " <<  e.what() << "\n" << endl;
  }
  cout << "Testing Luminosity Sensor..." << endl;
  try {
    testLuminositySensor();
  } catch (std::exception& e) {
    cout << "\nERROR: " << e.what() << "\n" << endl;
  }
  cout << "Testing 9DoF Board..." << endl;
  try {
    test9DoF();
  } catch (std::exception& e) {
    cout << "\nERROR: " << e.what() << "\n" << endl;
  }
  cout << "\n------TESTING MOCKING------\n" << endl;
  try {
    testMock();
  } catch (std::exception& e) {
    cout << "\nERROR: " << e.what()<< "\n" << endl;
  }
  return 0;
}

void test9DoF() {
  spacehauc_i2c::Adafruit9DOF sensor;
  sensor.accelerometer.init();
  sensor.magnetometer.init();
  sensor.gyroscope.init();
  sensor.printTable(100000, 10000);  // 20 data points, wait 10 ms between rows

  /* leaving commented code here for examples on how to work with each sensor
  spacehauc_i2c::Adafruit9DOF sensor;
  // test Accelerometer
  sensor.accelerometer.init();
  spacehauc_i2c::Triplet<double> acceleration;
  cout << "Initialized Accelerometer" << endl;
  cout << "Reading Acceleration Data..." << endl;
  for (int i = 0; i < 10; ++i) {
    cout << sensor.accelerometer.getName() << ": Avg. Acceleration = ";
    acceleration = sensor.accelerometer.readTriplet();
    cout << sensor.accelerometer.read() << "G, X = " << acceleration.getX() << "G, Y = "
         << acceleration.getY() << "G, Z = " << acceleration.getZ() << "G"
         << endl;
    usleep(500000);
  }

  sensor.magnetometer.init();
  spacehauc_i2c::Triplet<double> magField;
  cout << "Initialized Magnetometer" << endl;
  cout << "Reading Magnetic Field Data..." << endl;
  for(int i = 0; i < 50; ++i) {
    cout << sensor.magnetometer.getName() << ": Avg. Field = ";
    magField = sensor.magnetometer.readTriplet();
    cout << sensor.magnetometer.read() << " Gauss, X = " << magField.getX() << " Gauss"
         << ", Y = " << magField.getY() << " Gauss, Z = " << magField.getZ()
         << " Gauss" << endl;
    usleep(33334); // measured every 1/30 second
  }

  sensor.gyroscope.init();
  spacehauc_i2c::Triplet<double> degPerSec;
  cout << "Initialized Gyroscope" << endl;
  cout << "Reading Angular Velocity in Deg/Sec..." << endl;
  for(int i = 0; i < 100000; ++i) {
    cout << sensor.gyroscope.getName() << ": avg Velocity = ";
    degPerSec = sensor.gyroscope.readTriplet();
    cout << sensor.gyroscope.read() << " dps, X = " << degPerSec.getX() << " dps"
         << ", Y = " << degPerSec.getY() << " dps, Z = " << degPerSec.getZ()
         << " dps" << endl;
  }*/
}

void testTemperatureSensor() {
  spacehauc_i2c::MCP9808 tempSensor(0x18);
  tempSensor.init();
  cout << "Initialized Temperature Sensor" << endl;
  cout << "Reading Temperature data..." << endl;
  for (int i = 0; i < 5; ++i) {
    cout << tempSensor.getName() << ": Temperature = ";
    cout << tempSensor.read() << endl;
    usleep(500000);
  }
}

void testLuminositySensor() {
  spacehauc_i2c::TSL2561 light(0x39);
  light.init();
  cout << "Initialized Luminosity Sensor" << endl;
  cout << "Reading light intensity data..." << endl;
  for (int i = 0; i < 5; ++i) {
    cout << light.getName() << ": Luminosity = ";
    cout << light.read() << endl;
    sleep(1);
  }
}

void testMock() {
  int bus = 1;
  cout << "Initializing Mock Bus..." << endl;
  spacehauc_i2c_mock::I2C_Bus::init(bus);

  spacehauc_i2c_mock::MCP9808 tempSensor(0x18);
  tempSensor.init();
  cout << "Initialized Temperature Sensor" << endl;
  cout << "Reading Temperature data..." << endl;
  for (int i = 0; i < 5; ++i) {
    cout << tempSensor.getName() << ": Temperature = ";
    cout << tempSensor.read() << endl;
    usleep(500000);
  }

  spacehauc_i2c_mock::TSL2561 light(0x39);
  light.init();
  cout << "Initialized Luminosity Sensor" << endl;
  cout << "Reading light intensity data..." << endl;
  for (int i = 0; i < 5; ++i) {
    cout << light.getName() << ": Luminosity = ";
    cout << light.read() << endl;
    sleep(1);
  }
}
