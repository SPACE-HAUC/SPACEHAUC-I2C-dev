#include "spacehauc-i2c-dev.hpp"
#include <string>
#include <vector>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

using std::vector;
using std::string;
using std::cerr;
using std::cout;
using std::endl;
bool testTemperatureSensor();

int main(int argc, char* argv[]) {
  cout << "SPACEHAUC I2C Library Driver" << endl;
  cout << "Testing Temperature Sensor..." << endl;
  if (testTemperatureSensor()) {
    cout << "Success" << endl;
  } else {
    cout << "Failure" << endl;
  }
  return 0;
}

bool testTemperatureSensor() {
  uint8_t bus = 1;
  uint8_t address = 0x1d;
  uint8_t ID_register = 0x0F;
  uint8_t ctlRegister = 0x24;
  uint8_t dataRegister = 0x05;
  TemperatureSensor tempSensor(bus, address, ID_register, ctlRegister, dataRegister);
  if (tempSensor.initDevice() == false) {
    cerr << "Error: I2C bus failed to open." << endl;
    return false;
  }
  cout << "Opened I2C bus" << endl;
  if (tempSensor.initTempSensor() == false) {
    cerr << "Error: Temperature Sensor failed to initalize." << endl;
    return false;
  }
  cout << "Initialized Temperature Sensor" << endl;
  cout << "Reading Temperature data..." << endl;
  for (int i = 0; i < 5; i++) {
    cout << "Temperature = " << (int) tempSensor.readTemp() << endl;
    usleep(500000);
  }
  return true;
}
