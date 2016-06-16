#ifndef INCLUDE_SPACEHAUC-I2C-DEV_HPP_
#define INCLUDE_SPACEHAUC-I2C-DEV_HPP_

#include <string>
#include <vector>


using std::vector;
using std::string;

class I2C-Device { //eg 9dof sensor
protected:
  string mI2C_device_name;
  uint8_t mBus;
  int mFile;
  vector<uint8_t> mAddress(1);
  vector<uint8_t> mID_Regsiters(1);
  int readBytes(uint8_t count);
  int writeBytes;
public:
  I2C-Device();
  ~I2C-Device();
  bool initDevice();
};

class TemperatureSensor : public I2C-Device {
private:
  vector<uint8_t> mDataRegisters(1); //data registers
  vector<uint8_t> mControlRegisters(1); // control registers
  uint8_t mTemperature;
public:
  explicit TemperatureSensor(uint8_t bus, uint8_t address, uint8_t ID_register,
    uint8_t controlRegister, uint8_t dataRegister);
  ~TemperatureSensor();
  bool initTempSensor();
  int readTemp();
};

#endif  // INCLUDE_SPACEHAUC-I2C-DEV_HPP_
