# ADI Board Analog Device ADXL362  3-axis MEMS accelerometer
----
This is the library for the Analog Devices ADXL362 component and
it is an extension of the original Analog Device library.

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

Written by Seve <seve@ioteam.it>.

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

 Copyright 2012(c) Analog Devices, Inc.
 
  All rights reserved.
 
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   - Neither the name of Analog Devices, Inc. nor the names of its
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.
   - The use of this software may or may not infringe the patent rights
     of one or more patent holders.  This license does not release you
     from the requirement that you obtain separate licenses from these
     patent holders to use this software.
   - Use of the software either in source or binary form, must be run
     on or directly connected to an Analog Devices Inc. component.
 
  THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
