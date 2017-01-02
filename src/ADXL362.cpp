/***************************************************************************//**
 *   @file   ADXL362.c
 *   @brief  Implementation of ADXL362 Driver.
 *   @author DNechita(Dan.Nechita@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: $WCREV$
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "ADXL362.h"
#include <SPI.h>

/******************************************************************************/
/************************* Variables Declarations *****************************/
/******************************************************************************/
char selectedRange = 0;

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

//Read from or write to register from the SCP1000:
void
ADXL362::SPI_read(byte  thisRegister, unsigned char* pReadData,
                  int bytesToRead) 
{

    volatile unsigned char *data = pReadData;   // result to return

    _spi->beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
    
    // take the chip select low to select the device:
    digitalWrite(_ss, LOW);
    // send the device the register you want to read:
    _spi->transfer(*data);
    data++;
    bytesToRead--;
    // send the device the register you want to read:
    _spi->transfer(*data);
    data++;
    bytesToRead--;

    // if you still have another byte to read:
    while (bytesToRead > 0) {
        // shift the first byte left, then get the second byte:
        *data = _spi->transfer(0x00);
        // decrement the number of bytes left to read:
        bytesToRead--;
         data++;
    }

    // take the chip select high to de-select:
    digitalWrite(_ss, HIGH);
    
    _spi->endTransaction();
}

void
ADXL362::SPI_write(byte  thisRegister, unsigned char* pData, int bytesToWrite) {

    unsigned char* data = pData;
   
    _spi->beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0)); 
        
    // take the chip select low to select the device:
    digitalWrite(_ss, LOW);  
    // send the device the register you want to read:
    _spi->transfer(*data);
    bytesToWrite--;
            
    // if you still have another byte to read:
    while (bytesToWrite > 0) {
        // shift the first byte left, then get the second byte:
        data++;
        _spi->transfer(*data);
        // decrement the number of bytes left to read:
        bytesToWrite--;
    }
    // take the chip select high to de-select:
    digitalWrite(_ss, HIGH);
    
    _spi->endTransaction();
}

/******************************************************************************
 * @brief Checking Component presence reading the device ids.
 *
 * @return  0 - the device is present;
 *         false - an error occurred.
*******************************************************************************/
bool ADXL362::check(void)
{
    volatile unsigned char regValue = 0;
    
    getRegisterValue((unsigned char *)&regValue, ADXL362_REG_DEVID_AD, 1);
    if((regValue != ADXL362_DEVICE_AD))  {
        return false;
    } 
    getRegisterValue((unsigned char *)&regValue, ADXL362_REG_DEVID_MST, 1);
    if((regValue != ADXL362_DEVICE_MST))  {
        return false;
    }
    getRegisterValue((unsigned char *)&regValue, ADXL362_REG_PARTID, 1);
    if((regValue != ADXL362_PART_ID))  {
        return false;
    }   
    return true;
}

/******************************************************************************
 * @brief Initializes communication with the device and checks if the part is
 *        present by reading the device id.
 *
 * @return  0 - the initialization was successful and the device is present;
 *         -1 - an error occurred.
*******************************************************************************/
char ADXL362::begin(uint8_t ss, SPIClass *spi,  uint8_t irq)
{
    volatile char          status   = -1;
    
    this->_irq = irq;   
    this->_spi = spi;
    this->_ss = ss;

    pinMode(_ss, OUTPUT);
    digitalWrite(_ss, HIGH);
    pinMode(_irq, INPUT);
           
    // Write to SOFT RESET, "R"
    unsigned short reset = ADXL362_RESET_KEY;
    setRegisterValue(reset, ADXL362_REG_SOFT_RESET, 1);
    delay(10);
      
    if (check() == false) {
        return -1;
    }                    

    selectedRange = 2; // Measurement Range: +/- 2g (reset default).

    return status;
}

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerValue   - Data value to write.
 * @param registerAddress - Address of the register.
 * @param bytesNumber     - Number of bytes. Accepted values: 0 - 1.
 *
 * @return None.
*******************************************************************************/
void ADXL362::setRegisterValue(unsigned short registerValue,
                              unsigned char  registerAddress,
                              unsigned char  bytesNumber)
{
    unsigned char buffer[4] = {0, 0, 0, 0};

    buffer[0] = ADXL362_WRITE_REG;
    buffer[1] = registerAddress;
    buffer[2] = (registerValue & 0x00FF);
    buffer[3] = (registerValue >> 8);
    SPI_write(ADXL362_SLAVE_ID, buffer, bytesNumber + 2);
}

/***************************************************************************//**
 * @brief Performs a burst read of a specified number of registers.
 *
 * @param pReadData       - The read values are stored in this buffer.
 * @param registerAddress - The start address of the burst read.
 * @param bytesNumber     - Number of bytes to read.
 *
 * @return None.
*******************************************************************************/
void ADXL362::getRegisterValue(unsigned char* pReadData,
                              unsigned char  registerAddress,
                              unsigned char  bytesNumber)
{
    unsigned char buffer[32];
    unsigned char index = 0;
    
    buffer[0] = ADXL362_READ_REG;
    buffer[1] = registerAddress;
    for(index = 0; index < bytesNumber; index++)
    {
        buffer[index + 2] = pReadData[index];
    }
    SPI_read(ADXL362_SLAVE_ID, buffer, bytesNumber + 2);
    for(index = 0; index < bytesNumber; index++)
    {
        pReadData[index] = buffer[index + 2];
    }
}

/***************************************************************************//**
 * @brief Reads multiple bytes from the device's FIFO buffer.
 *
 * @param pBuffer     - Stores the read bytes.
 * @param bytesNumber - Number of bytes to read.
 *
 * @return None.
*******************************************************************************/
void ADXL362::getFifoValue(unsigned char* pBuffer, unsigned short bytesNumber)
{
    unsigned char  buffer[512];
    unsigned short index = 0;

    buffer[0] = ADXL362_WRITE_FIFO;
    for(index = 0; index < bytesNumber; index++)
    {
        buffer[index + 1] = pBuffer[index];
    }
    SPI_read(ADXL362_SLAVE_ID, buffer, bytesNumber + 1);
    for(index = 0; index < bytesNumber; index++)
    {
        pBuffer[index] = buffer[index + 1];
    }
}

/***************************************************************************//**
 * @brief Resets the device via SPI communication bus.
 *
 * @return None.
*******************************************************************************/
void ADXL362::softwareReset(void)
{
    setRegisterValue(ADXL362_RESET_KEY, ADXL362_REG_SOFT_RESET, 1);
}

/***************************************************************************//**
 * @brief Places the device into standby/measure mode.
 *
 * @param pwrMode - Power mode.
 *                  Example: 0 - standby mode.
 *                           1 - measure mode.
 *
 * @return None.
*******************************************************************************/
void ADXL362::setPowerMode(unsigned char pwrMode)
{
    unsigned char oldPowerCtl = 0;
    unsigned char newPowerCtl = 0;

    getRegisterValue(&oldPowerCtl, ADXL362_REG_POWER_CTL, 1);
    newPowerCtl = oldPowerCtl & ~ADXL362_POWER_CTL_MEASURE(0x3);
    newPowerCtl = newPowerCtl |
                  (pwrMode * ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_ON));
    setRegisterValue(newPowerCtl, ADXL362_REG_POWER_CTL, 1);
}

/***************************************************************************//**
 * @brief Places the device into wekup/no_wakup mode.
 *
 * @param wakeup - wakeup mode.
 *                 Example: 0 - no_wakeup mode.
 *                          1 - wakeup mode.
 *
 * @return None.
*******************************************************************************/
void ADXL362::setWakeupMode(bool wakeup)
{
    unsigned char oldPowerCtl = 0;
    unsigned char newPowerCtl = 0;

    getRegisterValue(&oldPowerCtl, ADXL362_REG_POWER_CTL, 1);
    newPowerCtl = oldPowerCtl & ~ADXL362_POWER_CTL_MEASURE(0x8);
    if (wakeup) {
        newPowerCtl = newPowerCtl | ADXL362_POWER_CTL_WAKEUP;
    }    
    setRegisterValue(newPowerCtl, ADXL362_REG_POWER_CTL, 1);
}
/***************************************************************************//**
 * @brief Selects the measurement range.
 *
 * @param gRange - Range option.
 *                  Example: ADXL362_RANGE_2G  -  +-2 g
 *                           ADXL362_RANGE_4G  -  +-4 g
 *                           ADXL362_RANGE_8G  -  +-8 g
 *                           
 * @return None.
*******************************************************************************/
void ADXL362::setRange(unsigned char gRange)
{
    unsigned char oldFilterCtl = 0;
    unsigned char newFilterCtl = 0;

    getRegisterValue(&oldFilterCtl, ADXL362_REG_FILTER_CTL, 1);
    newFilterCtl = oldFilterCtl & ~ADXL362_FILTER_CTL_RANGE(0x3);
    newFilterCtl = newFilterCtl | ADXL362_FILTER_CTL_RANGE(gRange);
    setRegisterValue(newFilterCtl, ADXL362_REG_FILTER_CTL, 1);
    selectedRange = (1 << gRange) * 2;
}

/***************************************************************************//**
 * @brief Selects the Output Data Rate of the device.
 *
 * @param outRate - Output Data Rate option.
 *                  Example: ADXL362_ODR_12_5_HZ  -  12.5Hz
 *                           ADXL362_ODR_25_HZ    -  25Hz
 *                           ADXL362_ODR_50_HZ    -  50Hz
 *                           ADXL362_ODR_100_HZ   -  100Hz
 *                           ADXL362_ODR_200_HZ   -  200Hz
 *                           ADXL362_ODR_400_HZ   -  400Hz
 *
 * @return None.
*******************************************************************************/
void ADXL362::setOutputRate(unsigned char outRate)
{
    unsigned char oldFilterCtl = 0;
    unsigned char newFilterCtl = 0;

    getRegisterValue(&oldFilterCtl, ADXL362_REG_FILTER_CTL, 1);
    newFilterCtl = oldFilterCtl & ~ADXL362_FILTER_CTL_ODR(0x7);
    newFilterCtl = newFilterCtl | ADXL362_FILTER_CTL_ODR(outRate);
    setRegisterValue(newFilterCtl, ADXL362_REG_FILTER_CTL, 1);
}

/***************************************************************************//**
 * @brief Reads the 3-axis raw data from the accelerometer.
 *
 * @param x - Stores the X-axis data(as two's complement).
 * @param y - Stores the Y-axis data(as two's complement).
 * @param z - Stores the Z-axis data(as two's complement).
 *
 * @return None.
*******************************************************************************/
void ADXL362::getXyz(int16_t* x, int16_t* y, int16_t* z)
{
    unsigned char xyzValues[6] = {0, 0, 0, 0, 0, 0};
    volatile uint16_t u16 = 0;
    
    getRegisterValue(xyzValues, ADXL362_REG_XDATA_L, 6);
    u16 = (xyzValues[1] << 8) + xyzValues[0];
    *x = (int16_t) u16;
    u16 = (xyzValues[3] << 8) + xyzValues[2];
    *y = (int16_t) u16;
    u16 = (xyzValues[5] << 8) + xyzValues[4];
    *z = (int16_t) u16;
}

/***************************************************************************//**
 * @brief Reads the 3-axis raw data from the accelerometer and converts it to g.
 *
 * @param x - Stores the X-axis data.
 * @param y - Stores the Y-axis data.
 * @param z - Stores the Z-axis data.
 *
 * @return None.
*******************************************************************************/
void ADXL362::getGxyz(float* x, float* y, float* z)
{
    unsigned char xyzValues[6] = {0, 0, 0, 0, 0, 0};
    volatile uint16_t u16 = 0;
    volatile int16_t s16 = 0;
    getRegisterValue(xyzValues, ADXL362_REG_XDATA_L, 6);

    u16 = (xyzValues[1] << 8) + xyzValues[0];
    s16 = (int16_t)u16;
    *x = ((float)s16) / (1000 / (selectedRange / 2));
    u16 = (xyzValues[3] << 8) + xyzValues[2];
    s16 = (int16_t)u16;
    *y = ((float)s16) /(1000 / (selectedRange / 2));
    u16 = (xyzValues[5] << 8) + xyzValues[4];
    s16 = (int16_t)u16;
    *z = ((float)s16) /(1000 / (selectedRange / 2));
}

/***************************************************************************//**
 * @brief Reads the temperature of the device.
 *
 * @return tempCelsius - The value of the temperature(degrees Celsius).
*******************************************************************************/
float ADXL362::readTemperature(void)
{
    unsigned char     rawTempData[2] = {0, 0};
    float             tempCelsius    = 0;
    volatile uint16_t unsignedTemp = 0;
    volatile int16_t  signedTemp = 0;
    
    getRegisterValue(rawTempData, ADXL362_REG_TEMP_L, 2);
    unsignedTemp = (rawTempData[1] << 8) + rawTempData[0];
    signedTemp = (int16_t)unsignedTemp;   
    tempCelsius = (float)signedTemp * 0.065;
    
    return tempCelsius;
}

/***************************************************************************//**
 * @brief Configures the FIFO feature.
 *
 * @param mode         - Mode selection.
 *                       Example: ADXL362_FIFO_DISABLE      -  FIFO is disabled.
 *                                ADXL362_FIFO_OLDEST_SAVED -  Oldest saved mode.
 *                                ADXL362_FIFO_STREAM       -  Stream mode.
 *                                ADXL362_FIFO_TRIGGERED    -  Triggered mode.
 * @param waterMarkLvl - Specifies the number of samples to store in the FIFO.
 * @param enTempRead   - Store Temperature Data to FIFO.
 *                       Example: 1 - temperature data is stored in the FIFO
 *                                    together with x-, y- and x-axis data.
 *                                0 - temperature data is skipped.
 *
 * @return None.
*******************************************************************************/
void ADXL362::fifoSetup(unsigned char  mode,
                       unsigned short waterMarkLvl,
                       unsigned char  enTempRead)
{
    unsigned char writeVal = 0;

    writeVal = ADXL362_FIFO_CTL_FIFO_MODE(mode) |
               (enTempRead * ADXL362_FIFO_CTL_FIFO_TEMP) |
               ADXL362_FIFO_CTL_AH;
    setRegisterValue(writeVal, ADXL362_REG_FIFO_CTL, 1);
    setRegisterValue(waterMarkLvl, ADXL362_REG_FIFO_SAMPLES, 2);
}

/***************************************************************************//**
 * @brief Configures activity detection.
 *
 * @param refOrAbs  - Referenced/Absolute Activity Select.
 *                    Example: 0 - absolute mode.
 *                             1 - referenced mode.
 * @param threshold - 11-bit unsigned value that the adxl362 samples are
 *                    compared to.
 * @param time      - 8-bit value written to the activity timer register. The 
 *                    amount of time (in seconds) is: time / ODR, where ODR - is 
 *                    the output data rate.
 *
 * @return None.
*******************************************************************************/
void ADXL362::setupActivityDetection(unsigned char  refOrAbs,
                                    unsigned short threshold,
                                    unsigned char  time)
{
    unsigned char oldActInactReg = 0;
    unsigned char newActInactReg = 0;

    /* Configure motion threshold and activity timer. */
    setRegisterValue((threshold & 0x7FF), ADXL362_REG_THRESH_ACT_L, 2);
    setRegisterValue(time, ADXL362_REG_TIME_ACT, 1);
    /* Enable activity interrupt and select a referenced or absolute
       configuration. */
    getRegisterValue(&oldActInactReg, ADXL362_REG_ACT_INACT_CTL, 1);
    newActInactReg = oldActInactReg & ~ADXL362_ACT_INACT_CTL_ACT_REF;
    newActInactReg |= ADXL362_ACT_INACT_CTL_ACT_EN |
                     (refOrAbs * ADXL362_ACT_INACT_CTL_ACT_REF);
    setRegisterValue(newActInactReg, ADXL362_REG_ACT_INACT_CTL, 1);
}

/***************************************************************************//**
 * @brief Configures inactivity detection.
 *
 * @param refOrAbs  - Referenced/Absolute Inactivity Select.
 *                    Example: 0 - absolute mode.
 *                             1 - referenced mode.
 * @param threshold - 11-bit unsigned value that the adxl362 samples are
 *                    compared to.
 * @param time      - 16-bit value written to the inactivity timer register. The 
 *                    amount of time (in seconds) is: time / ODR, where ODR - is  
 *                    the output data rate.
 *
 * @return None.
*******************************************************************************/
void ADXL362::setupInactivityDetection(unsigned char  refOrAbs,
                                      unsigned short threshold,
                                      unsigned short time)
{
    unsigned char oldActInactReg = 0;
    unsigned char newActInactReg = 0;
    
    /* Configure motion threshold and inactivity timer. */
    setRegisterValue((threshold & 0x7FF), ADXL362_REG_THRESH_INACT_L, 2);
    setRegisterValue(time, ADXL362_REG_TIME_INACT_L, 2);
    /* Enable inactivity interrupt and select a referenced or absolute
       configuration. */
    getRegisterValue(&oldActInactReg, ADXL362_REG_ACT_INACT_CTL, 1);
    newActInactReg  = oldActInactReg & ~ADXL362_ACT_INACT_CTL_INACT_REF;
    newActInactReg |= ADXL362_ACT_INACT_CTL_INACT_EN |
                      (refOrAbs * ADXL362_ACT_INACT_CTL_INACT_REF);
    setRegisterValue(newActInactReg, ADXL362_REG_ACT_INACT_CTL, 1);
}


/***************************************************************************//**
 * @brief Configures the Awake map for INT1/INT2.
 *
 * @param awakeMap  - awakeMap bitmask
 *
 * ADXL362_INTMAP2_INT_LOW
 * ADXL362_INTMAP2_AWAKE   
 * ADXL362_INTMAP2_INACT
 * ADXL362_INTMAP2_ACT
 * ADXL362_INTMAP2_FIFO_OVERRUN
 * ADXL362_INTMAP2_FIFO_WATERMARK
 * ADXL362_INTMAP2_FIFO_READY
 * ADXL362_INTMAP2_DATA_READY
 *
 * @return None.
*******************************************************************************/
void ADXL362::setIntMap2(unsigned char  awakeMap)
{
    setRegisterValue(awakeMap, ADXL362_REG_INTMAP2, 1);
}
void ADXL362::setIntMap1(unsigned char  awakeMap)
{
    setRegisterValue(awakeMap, ADXL362_REG_INTMAP1, 1);
}

ADXL362 adiAccelerometer;