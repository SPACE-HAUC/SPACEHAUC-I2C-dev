<!-- Replace the below line with your own Travis-CI build label. -->
[![Build Status](https://travis-ci.org/SpaceHAUC-Command-and-Data-Handling/SPACEHAUC-I2C-dev.svg?branch=master)](https://travis-ci.org/SpaceHAUC-Command-and-Data-Handling/SPACEHAUC-I2C-dev)

[![Managed with Taiga.io](https://camo.githubusercontent.com/eec9589abe09569dc4a1706b36527b49051b89db/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f6d616e61676564253230776974682d54616967612e696f2d677265656e2e737667)](https://taiga.io "Managed with Taiga.io")

## SPACEHAUC I2C DEV TOOLS ALPHA v0.4.1.0

This is the repository for SPACEHAUC-I2C-dev code audit. 

This is a library managed by the UMass Lowell SPACE HAUC Command and Data Handling Team. Its various functions should make using I2C in Linux on an Intel Edison much easier. Currently in Alpha release, with support for temperature sensors, magnetometers, luminosity sensors, and a pulse width modulation board. More sensors are planned in the future.

#### Using This Library
This i2c header file and implementation file, when placed inside your include and src folders of cmake (respectively) should greatly simplify the use of I2C. Instead of complex function calls and weird arrays, our I2C library first requires you to initialize the hardware bus that your i2c device will be communicating over and give it a file to write to:

```C++
int file;
initBus (int bus, int *file);
```

Then you need to create an object with an overloaded constructor that includes all the data required by your device. If you are using a device that is well supported by our library, it should be as simple as ClassName ObjectName(file,address,IDregister, control register, dataRegister). For example, an object for the temperature sensor included on the 9 Degrees of Freedom board can be created with the line :

```C++
TemperatureSensor tempSensor(file, 0x1d, 0x0F, 0x24, 0x05);
```

Then, by calling the method initTempSensor on the object, you can prepare the tempSensor to be read. initTempeSensor returns a boolean, true if the initialization succeeded, false if it failed, which can be used to error check, but this is unnescesary right now. Other supported sensors have init methods, just look through the source code for your sensor.

```C++
tempSensor.initTempSensor()
```

Once the sensor is initialized, all that’s left to do is read from it. For the temperature sensor, this can be done with the method readTemp, but all other supported sensors have a read function. Check the source code. This function returns a double, which you can print or store somewhere else.

```C++
data = tempSensor.readTemp();
std::cout << data;
```
*If you notice a problem with the initMagnetometer function, this may be due to the fact that the same register is written to twice.*



[![GPL License](http://darrienglasser.com/gpl-v3-logo.jpg)](http://www.gnu.org/licenses/gpl-3.0.en.html)
