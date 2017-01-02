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

bool led = false;
#define TRIGGER_LED  digitalWrite(PIN_LED, led ? 225 : 0); \
                     led =!led

void setup(void)
{
    Serial.begin(115200);
        
    SPI1.begin();

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    
    adiAccelerometer.begin(ADXL362_CS, &SPI1, ADXL362_EINT);
    adiAccelerometer.setMeasurementMode();
        
    // Waiting for the console to continue
    while (!Serial) {
        ;
    }
}



volatile uint32_t i = 0;
float x, y, z;
int16_t  x16, y16, z16;
float temp;

void loop(void)
{    
    if (!(i%5)) {
        TRIGGER_LED;

        if (!(i%500)) {
            temp = adiAccelerometer.readTemperature();
            Serial.print("\nADXL362 Device Temperature = ");
            Serial.println(temp);
            Serial.print("\n");
        }   
   
        adiAccelerometer.getGxyz(&x, &y, &z);
        adiAccelerometer.getXyz(&x16, &y16, &z16);        
          
        Serial.print("  X / Y / Z =  ");
        Serial.print(x);
        Serial.print("  /  ");
        Serial.print(y);        
        Serial.print("  /  ");
        Serial.print(z);     

        Serial.print("   RAW: [  ");
        Serial.print(x16);
        Serial.print("  /  ");
        Serial.print(y16);
        Serial.print("  /  ");
        Serial.print(z16);   
        Serial.println("  ]"); 
    }   
    ++i;
    delay(10);
}