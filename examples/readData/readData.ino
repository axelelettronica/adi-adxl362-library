/*
    ADI ADXL3262 Library - get accelerometer information

    This example shows how to retrieve the x,y,z values 
    from the ADXL362 accelrometer

    Created 2 November 2016 by Seve (seve@ioteam.it)

    This example is in the public domain
    https://github.com/axelelettronica

    more information available here: http://www.analog.com/ADXL362
 */


#include <Arduino.h>
#include <SPI.h>
#include <ADXL362.h>

volatile uint16_t i = 0;
float x, y, z;
float temp;


void setup(void)
{
    SerialUSB.begin(115200);

    SPI1.begin();
    delay(1000);

    // Stop the execution till a Serial console is connected
    while (!SerialUSB) {
        ;
    }

   adiAccelerometer.begin(ADXL362_CS, &SPI1, ADXL362_EINT);
   adiAccelerometer.setPowerMode(1);
}




void loop(void)
{    
    digitalWrite(PIN_LED, HIGH);

    if (i%10000) {

        adiAccelerometer.getGxyz(&x, &y, &z);
        temp = adiAccelerometer.readTemperature();
        
        SerialUSB.print("Temperature = ");
        SerialUSB.print(temp);
        SerialUSB.print("\tX axis = ");
        SerialUSB.print(x);
        SerialUSB.print("\tY axis = ");
        SerialUSB.print(y);        
        SerialUSB.print("\tz axis = ");
        SerialUSB.println(z);     
    }   

    ++i;
    delay(100);
}
