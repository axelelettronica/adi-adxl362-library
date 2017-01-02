/*
    ADI ADXL3262 Library - Motion detection 

    This example shows how to get informed about a motion detection from the
    ADXL362 accelerometer.

    Created 2 November 2016 by Seve (seve@ioteam.it)

    This example is in the public domain
    https://github.com/axelelettronica

    more information available here: http://www.analog.com/ADXL362
 */

#include <Arduino.h>
#include <SPI.h>
#include <ADXL362.h>

bool printOut = true;

int led = 1;
uint32_t i = 0;
float x, y, z;
int16_t  x16, y16, z16;
char interrupt;
char prev_interrupt;


void SET_LED( int x)  
{
    if (led != x) {
        digitalWrite(PIN_LED, x ? 225 : 0);
        led = x;
    }               
}

void setup(void)
{
        Serial.begin(115200);
        
        SPI1.begin();

        pinMode(PIN_LED, OUTPUT);
        digitalWrite(PIN_LED, HIGH);

        adiAccelerometer.begin(ADXL362_CS, &SPI1, ADXL362_EINT);
   
        /*! Configures inactivity detection. */
        // Activity threshold = 250, with 0 consecutive events all enabled.
        adiAccelerometer.setupActivityDetection(0x3F, 250, 0);

        /*! Configures inactivity detection. */
        // Inactivity threshold = 150, with 3 consecutive events all enabled.
        adiAccelerometer.setupInactivityDetection(0x3F, 150, 3);
           
        // map awake bit to Int2 
        adiAccelerometer.setIntMap1(ADXL362_INTMAP2_AWAKE);

        adiAccelerometer.setWakeupMode(true);                    
        adiAccelerometer.setMeasurementMode();
        
        // Waiting for the console to continue
        while (!Serial) {
            ;
        }                    
        Serial.println("Initial setup done!");
}



void loop(void)
{    
    if (!(i%5)) {
        prev_interrupt = interrupt;
        
        adiAccelerometer.getGxyz(&x, &y, &z);
        adiAccelerometer.getXyz(&x16, &y16, &z16);        
        interrupt = digitalRead(ADXL362_EINT);
        
        SET_LED(interrupt);        
        
        if (prev_interrupt != interrupt) {
            if (printOut) {
                Serial.print("X / Y / Z = ");
                Serial.print(x);
                Serial.print(" / ");
                Serial.print(y);        
                Serial.print(" / ");
                Serial.print(z);     

                Serial.print("  RAW: [ ");
                Serial.print(x16);
                Serial.print(" / ");
                Serial.print(y16);
                Serial.print(" / ");
                Serial.print(z16);   
                Serial.print(" ] "); 
            }            
            Serial.print((interrupt) ? "Motion detected" : "Stopped");
            Serial.println("\n");	       
        }
    }   
    ++i;
    delay(50);
}

