// Copyright 2016 - 2017 David Baumann and Jacob Hempel, UMass Lowell C&DH Team

#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>
#include "../include/spacehauc-i2c-dev.h"
#include "../include/spacehauc-i2c-mock.h"

using std::vector;
using std::string;
using std::cerr;
using std::cout;
using std::endl;

void testTemperatureSensor();
void testLuminositySensor();
void testMock();

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
  cout << "\n------TESTING MOCKING------\n" << endl;
  try {
    testMock();
  } catch (std::exception& e) {
    cout << "\nERROR: " << e.what()<< "\n" << endl;
  }
  return 0;
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
