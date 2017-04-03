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

using std::vector;
using std::string;
using std::cerr;
using std::cout;
using std::endl;

using spacehauc_i2c::I2C_Bus;
using spacehauc_i2c::TSL2561;
using spacehauc_i2c::MCP9808;

bool testTemperatureSensor();
bool testLuminositySensor();

int main(int argc, char* argv[]) {
  cout << "SPACEHAUC I2C Library Driver" << endl;
  cout << "Initializing Bus" << endl;
  int bus = 1;  // the first i2c bus
  if (I2C_Bus::init(bus) == false) {
    cerr << "Error: I2C bus failed to open." << endl;
  }
  cout << "Testing Temperature Sensor..." << endl;
  if (testTemperatureSensor()) {
    cout << "Success" << endl;
  } else {
    cout << "Failure" << endl;
  }
  cout << "Testing Luminosity Sensor..." << endl;
  if (testLuminositySensor()) {
    cout << "Success" << endl;
  } else {
    cout << "Failure" << endl;
  }
  return 0;
}

bool testTemperatureSensor() {
  /*
  uint8_t address = 0x1d;
  uint8_t ID_register = 0x0F;
  uint8_t ctlRegister = 0x24;
  uint8_t dataRegister = 0x05;
  */
  MCP9808 tempSensor(0x18);
  if (tempSensor.init() == false) {
    cerr << "Error: Temperature Sensor failed to initalize." << endl;
    return false;
  }
  cout << "Initialized Temperature Sensor" << endl;
  cout << "Reading Temperature data..." << endl;
  for (int i = 0; i < 5; ++i) {
    cout << tempSensor.getName() << ": Temperature = ";
    cout << tempSensor.read() << endl;
    usleep(500000);
  }
  return true;
}

bool testLuminositySensor() {
  TSL2561 light(0x39);
  if (light.init() == false) {
    cerr << "Error: Luminosity Sensor failed to initialize." << endl;
    return false;
  }
  cout << "Initialized Luminosity Sensor" << endl;
  cout << "Reading light intensity data..." << endl;
  for (int i = 0; i < 5; ++i) {
    cout << light.getName() << ": Luminosity = " << light.read() << endl;
    sleep(1);
  }
  return true;
}
