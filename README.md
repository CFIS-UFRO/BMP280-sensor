This code allows to use the BMP280 sensor for measuring pressure and temperature (you can add easily altitude measuring) on the Raspberry Pi board.   
It is written in c++ using the wiringPi library.

# Example
main.cpp:
```
#include <iostream>
#include "rpi_bmp280.h"
#include "rpi_tca9548a.h"

int main(void){
  rpi_bmp280 bmp;
  bmp.init(0x76);
  double p = bmp.read_pressure();
  double t = bmp.read_temperature();
  std::cout << "Pressure (Pa)   : " << p << std::endl;
  std::cout << "Temperature (°C): " << t << std::endl;
}
```
To compile: g++ main.cpp rpi_bmp280.h rpi_bmp280.cpp -o main -lwiringPi

# Available functions
The user can access to the following functions:

### int init(int id)
Initialize the sensor in normal mode.
- id: I2C address.

### void config(int p_overscan, int t_overscan, int mode, int standby, int filter)
Set the operating mode and configuration.
All input parameters are indexes.
- p_overscan: index of the pressure overscan property: 0: disable, 1: overscan x1, 2: overscan x2, 3: overscan x4, 4: overscan x8, 5: overscan x16.
- p_overscan: index of the temperature overscan property: 0: disable, 1: overscan x1, 2: overscan x2, 3: overscan x4, 4: overscan x8, 5: overscan x16.
- mode: index of the operating mode: 0: sleep, 1: force, 2: normal.
- standby: standby time: 0: 0.5ms, 1: 62.5ms, 2: 125ms, 3: 250ms, 4: 500ms, 5: 1000ms, 6: 2000ms, 7: 4000ms.
- filter: filter mode: 0: IIR filter disable, 1: IIR filter x2, 2: IIR filter x4, 3: IIR filter x8, 4: IIR filter x16.

Default configuration: config(1,1,2,0,0).

### double read_temperature()
Return the temperature in °C.

### double read_pressure()
Return the pressure in Pa.
