/***************************************************************************//**
 *   @file   ADXL362.h
 *   @brief  Header file of ADXL362 Driver.
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

#ifndef __ADXL362_H__
#define __ADXL362_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <Arduino.h>
#include "ADXL362Reg.h"

/******************************************************************************/
/********************************* ADXL362 ************************************/
/******************************************************************************/


class ADXL362
{
    private:
    uint8_t _address;
    uint8_t  _clk, _miso, _mosi, _ss, _irq;

    public:
    // ADXL362() {}
    ADXL362(uint8_t _sck, uint8_t _miso,uint8_t _mosi, uint8_t _ss, uint8_t _irq = 0);
    ~ADXL362() {}

    protected:
    void SPI_read(byte  thisRegister, unsigned char* pReadData, int bytesToRead);
    void SPI_write(byte  thisRegister, unsigned char* pData, int bytesToWrite);
    
    public:
    char begin(void);

    /*! Writes data into a register. */
    void setRegisterValue(unsigned short registerValue,
                          unsigned char  registerAddress,
                          unsigned char  bytesNumber);

    /*! Performs a burst read of a specified number of registers. */
    void getRegisterValue(unsigned char *pReadData,
                          unsigned char  registerAddress,
                          unsigned char  bytesNumber);

    /*! Reads multiple bytes from the device's FIFO buffer. */
    void getFifoValue(unsigned char *pBuffer, unsigned short bytesNumber);

    /*! Resets the device via SPI communication bus. */
    void softwareReset(void);

    /*! Places the device into standby/measure mode. */
    void setPowerMode(unsigned char pwrMode);

    /*! Selects the measurement range. */
    void setRange(unsigned char gRange);

    /*! Selects the Output Data Rate of the device. */
    void setOutputRate(unsigned char outRate);

    /*! Reads the 3-axis raw data from the accelerometer. */
    void getXyz(short *x, short *y, short *z);

    /*! Reads the 3-axis raw data from the accelerometer and converts it to g. */
    void getGxyz(float* x, float* y, float* z);

    /*! Reads the temperature of the device. */
    float readTemperature(void);

    /*! Configures the FIFO feature. */
    void fifoSetup(unsigned char  mode,  unsigned short waterMarkLvl,
                   unsigned char  enTempRead);

    /*! Configures activity detection. */
    void setupActivityDetection(unsigned char  refOrAbs,
                                unsigned short threshold,
                                unsigned char  time);

    /*! Configures inactivity detection. */
    void setupInactivityDetection(unsigned char  refOrAbs,
                                  unsigned short threshold,
                                  unsigned short time);
};

#endif /* __ADXL362_H__ */
