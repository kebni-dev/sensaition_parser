# kebni_driver

A parser for binary data from the [**Kebni SensAItion**](https://www.kebni.com/) IMU/AHRS/INS sensors. 
It reads binary data and decodes it into physically meaningful measurements as defined the SensAItion User Manual (D0000447) from Kebni AB.
Please contact Kebni for access to the User Manual and for any support questions.

## How to build
```bash
cd kebni_driver
mkdir build
cd build
cmake ..
cmake --build .
```

## How to include in a CMake project
Include the following in the ```CMakeLists.txt``` file of your project:

```bash
add_subdirectory(external/kebni-driver)
add_executable(my_program main.cpp)
target_link_libraries(my_program PRIVATE Kebni::driver)
```

## How to run unit tests
How to build the project with the unit tests and run them:

```bash
cd kebni_driver
mkdir build
cd build
cmake .. -DKEBNI_DRIVER_BUILD_TESTS=ON
cmake --build .
test/test_parser
```

## Typical usage
A typical usage would be to connect the Data UART of a SensAItion INS sensor to a serial port of your system and configure it with the following Data UART configuration string:

```
o0002s484204214224230000010020030100110120130200210220230300310320330400410420430500510520533103113123133203213223233303313323333403413423434C04C14C24C34D04D14D24D34E04E14E24E34F04F14F24F35905915925935A05A15A25A35B05B15B25B3X
```

This string defines which measurements are included in the binary message, and in which order. Then one can create a class that inherits from ```KebniDriver```, configure it with the same string and implement the ```onMeasurements()``` callback to handle any parsed messages. When data arrives on the serial port, it is fed to ```processByte()``` to be parsed.