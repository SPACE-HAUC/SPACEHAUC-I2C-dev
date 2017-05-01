<!-- Replace the below line with your own Travis-CI build label. -->
[![Build Status](https://travis-ci.org/SpaceHAUC-Command-and-Data-Handling/SPACEHAUC-I2C-dev.svg?branch=master)](https://travis-ci.org/SpaceHAUC-Command-and-Data-Handling/SPACEHAUC-I2C-dev) [![Managed with Taiga.io](https://camo.githubusercontent.com/eec9589abe09569dc4a1706b36527b49051b89db/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f6d616e61676564253230776974682d54616967612e696f2d677265656e2e737667)](https://taiga.io "Managed with Taiga.io")

## SPACEHAUC I2C DEV TOOLS API FLAVESCENT RELEASE (ALPHA v0.6.0.0)

This is a library managed by the UMass Lowell SPACE HAUC Command and Data Handling Team. Its various functions should make using I2C in Linux on the raspberry pi much easier. Currently in Alpha release, with support for temperature sensors and luminosity sensors. The library should be easily expandable to accommodate more sensors in a future release.

#### Using This Library

This i2c header file and implementation file, when placed inside your include and src folders of cmake (respectively) should greatly simplify the use of I2C.

```C++
#include "spacehauc-i2c-dev.h"
using namespace spacehauc_i2c;  // yes I know this is bad, this is just easier to show for an example.
```  

Instead of complex function calls and weird arrays, our I2C library first requires you to initialize the hardware bus that your i2c device will be communicating over:

```C++
int bus = 1;
if (I2C_Bus::init(bus) == false) {
  cerr << "Error: I2C bus failed to open." << endl;
}
```

Then you need to create an object with an overloaded constructor that includes all the data required by your device. If you are using a device that is well supported by our library, it should be as simple as ClassName ObjectName(address). For example, an object for the Adafruit MCP9808 temperature sensor can be created with the line :

```C++
MCP9808 tempSensor(0x18);
```

Then, by calling the method .init() on the object, you can prepare the tempSensor to be read. .init() returns a boolean, true if the initialization succeeded and false if it failed, which can be used to error check. Other supported devices will have a .init() member function for setting up the device.

```C++
if (tempSensor.init() == false) {
  cerr << "Error: Temperature Sensor failed to initalize." << endl;
}
```

Once the sensor is initialized, data can be read from it. For most sensors the .read() method will return the data that the sensor reads. The MCP9808 temperature sensor will return the current temperature in Celsius. The .read() function returns a double.

```C++
double data = tempSensor.read();
```

The sensors also have a .getName() method, that returns as a string the sensor type in addition to its address on the bus.

```C++
string name = tempSensor.getName();
```

When put all together, output can be nicely formatted as:

```C++
cout << "Temperature Sensor " << name << ": " << data << " C" << endl;
```
This would output:
```
Temperature Sensor MCP9808_0x18: 26.5 C
```

### Adafruit9DOF
The newest sensor added to the library is the Adafruit 9DoF board. This board has essentially three sensors on it, an Accelerometer, a Magnetometer, and a Gyroscope. Each sensor is included in the class Adafruit9DOF:
```C++
Adafruit9DOF sensor;
```







### Mocking

Have some code that uses this library but no physical sensors? Simply change your include directive and your namespace:

```C++
#include "spacehauc-i2c-mock.h"
using namespace spacehauc_i2c_mock; // yes I know this is bad, this is just easier to show for an example.
```

This will allow your program to still run correctly, and the mocking library will assume that I2C communications are working, and fake I2C output will be generated from the code.

[![SPACEHAUC Logo](http://djbaumann.github.io/images/spacehauclogo.png)](https://www.uml.edu/Research/LoCSST/Research/spacehauc/about.aspx)[![GPL License](http://darrienglasser.com/gpl-v3-logo.jpg)](http://www.gnu.org/licenses/gpl-3.0.en.html)
