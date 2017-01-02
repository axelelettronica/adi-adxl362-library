# ADI Board Analog Device ADXL362  3-axis MEMS accelerometer
----
This is the library for the Analog Devices ADXL362 component and
it is an extension of the original Analog Device library for the component.

The ADXL362 is an ultralow power, 3-axis MEMS accelerometer that consumes less than 2 μA at a 100 Hz output data rate
and 270 nA when in motion triggered wake-up mode.

Measurement ranges of ±2 g, ±4 g, and ±8 g are available, with a resolution of 1 mg/LSB on the ±2 g range. 

In addition to its ultralow power consumption, the ADXL362 has many features to enable true system level power reduction. 
It includes a deep multimode output FIFO, a built-in micropower temperature sensor, 
and several activity detection modes including adjustable threshold sleep and wake-up 
operation that can run as low as 270 nA at a 6 Hz (approximate) measurement rate. 
A pin output is provided to directly control an external switch when activity is detected, 
if desired. 
In addition, the ADXL362 has provisions for external control of sampling time and/or an external clock.


[* ADXL362 Home Page *](http://www.analog.com/ADXL362)

Host control and result reading is performed using an SPI interface, no extra pin are required.

It was principally designed to work with the ADI board, but could
be easily adapt and use on every Arduino and Arduino Certified boards.

Written by Seve <seve@axelelettronica>.

## Repository Contents
-------------------
* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE. 
* **/src** - Source files for the library (.cpp, .h).
* **library.properties** - General library properties for the Arduino package manager.

## Releases
---
#### v1.0.0 First Release

## Documentation
--------------
* **[Installing an Arduino Library Guide](http://www.arduino.cc/en/Guide/Libraries#toc3)** - How to install a SmartEverything library on the Arduino IDE using the Library Manager


##  Information
-------------------

Copyright (c) Amel Technology. All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

