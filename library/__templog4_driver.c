/*
    __templog4_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__templog4_driver.h"
#include "__templog4_hal.c"

/* ------------------------------------------------------------------- MACROS */



/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __TEMPLOG4_DRV_I2C__
static uint8_t _slaveAddress;
static uint8_t _slaveEEPROM;
#endif

const uint8_t _TEMPLOG4_SLAVE_ADDR_A0_H                    = 0x19;
const uint8_t _TEMPLOG4_SLAVE_ADDR_A0_L                    = 0x18;

const uint8_t _TEMPLOG4_CAPABILITY_REG                     = 0x00;
const uint8_t _TEMPLOG4_CONFIG_REG                         = 0x01;
const uint8_t _TEMPLOG4_TEMP_UPPER_REG                     = 0x02;
const uint8_t _TEMPLOG4_TEMP_LOWER_REG                     = 0x03;
const uint8_t _TEMPLOG4_TEMP_CRITICAL_REG                  = 0x04;
const uint8_t _TEMPLOG4_TEMP_AMBIENT_REG                   = 0x05;
const uint8_t _TEMPLOG4_MANUFACT_ID_REG                    = 0x06;
const uint8_t _TEMPLOG4_DEVICE_ID_REG                      = 0x07;
const uint8_t _TEMPLOG4_SMBUS_REG                          = 0x22;

const uint8_t _TEMPLOG4_EVENT_SHDN_STATUS_MASK             = 0x80;
const uint8_t _TEMPLOG4_BUS_TIMEOUT_STATUS_MASK            = 0x40;
const uint8_t _TEMPLOG4_HIGH_VOLT_INPUT_STATUS_MASK        = 0x20;
const uint8_t _TEMPLOG4_RESOLUTION_STATUS_MASK             = 0x18;
const uint8_t _TEMPLOG4_MEAS_RANGE_STATUS_MASK             = 0x04;
const uint8_t _TEMPLOG4_ACCURACY_STATUS_MASK               = 0x02;
const uint8_t _TEMPLOG4_ALARM_STATUS_MASK                  = 0x01;

const uint16_t _TEMPLOG4_TLIMIT_HYST_0_DEG                 = 0x0000;
const uint16_t _TEMPLOG4_TLIMIT_HYST_ONE_HALF_DEG          = 0x0200;
const uint16_t _TEMPLOG4_TLIMIT_HYST_3_DEG                 = 0x0400;
const uint16_t _TEMPLOG4_TLIMIT_HYST_6_DEG                 = 0x0600;
const uint16_t _TEMPLOG4_CONT_CONV_MODE                    = 0x0000;
const uint16_t _TEMPLOG4_SHUTDOWN_MODE                     = 0x0100;
const uint16_t _TEMPLOG4_TCRIT_LOCKED                      = 0x0080;
const uint16_t _TEMPLOG4_TUPPER_TLOWER_LOCKED              = 0x0040;
const uint16_t _TEMPLOG4_INT_CLEAR                         = 0x0020;
const uint16_t _TEMPLOG4_EVENT_OUTPUT_STATUS_MASK          = 0x0010;
const uint16_t _TEMPLOG4_EVENT_OUTPUT_EN                   = 0x0008;
const uint16_t _TEMPLOG4_EVENT_ALL_TLIMIT                  = 0x0000;
const uint16_t _TEMPLOG4_EVENT_TCRIT_ONLY                  = 0x0004;
const uint16_t _TEMPLOG4_EVENT_POL_ACT_LOW                 = 0x0000;
const uint16_t _TEMPLOG4_EVENT_POL_ACT_HIGH                = 0x0002;
const uint16_t _TEMPLOG4_EVENT_COMPARATOR_MODE             = 0x0000;
const uint16_t _TEMPLOG4_EVENT_INTERRUPT_MODE              = 0x0001;

const uint8_t _TEMPLOG4_TCRIT_DETECT                       = 0x80;
const uint8_t _TEMPLOG4_TUPPER_DETECT                      = 0x40;
const uint8_t _TEMPLOG4_TLOWER_DETECT                      = 0x20;
const uint8_t _TEMPLOG4_NBYTES_ERROR                       = 0x04;
const uint8_t _TEMPLOG4_TEMP_RANGE_ERROR                   = 0x03;
const uint8_t _TEMPLOG4_ADDR_ERROR                         = 0x02;
const uint8_t _TEMPLOG4_ALARMING                           = 0x01;
const uint8_t _TEMPLOG4_OK                                 = 0x00;

const uint8_t _TEMPLOG4_SMBUS_TIMEOUT_DIS                  = 0x80;
const uint8_t _TEMPLOG4_SMBUS_TIMEOUT_SHTDN_EN             = 0x20;
const uint8_t _TEMPLOG4_SMBUS_ARA_DIS                      = 0x01;

const uint8_t _TEMPLOG4_EEPROM_WRITE                       = 0x00;
const uint8_t _TEMPLOG4_SW_WRITE_PROTECT                   = 0x01;
const uint8_t _TEMPLOG4_CLEAR_WRITE_PROTECT                = 0x02;

const uint16_t _TEMPLOG4_EEPROM_SIZE                       = 256;
const uint16_t _TEMPLOG4_MANUFACT_ID                       = 0x1131;
const uint16_t _TEMPLOG4_DEVICE_ID                         = 0xA203;
const uint8_t _TEMPLOG4_DUMMY_BYTE                         = 0x00;

/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */



/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */



/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __TEMPLOG4_DRV_SPI__

void templog4_spiDriverInit(T_TEMPLOG4_P gpioObj, T_TEMPLOG4_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __TEMPLOG4_DRV_I2C__

void templog4_i2cDriverInit(T_TEMPLOG4_P gpioObj, T_TEMPLOG4_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );
    
    
}

#endif
#ifdef   __TEMPLOG4_DRV_UART__

void templog4_uartDriverInit(T_TEMPLOG4_P gpioObj, T_TEMPLOG4_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif

/* ----------------------------------------------------------- IMPLEMENTATION */

T_TEMPLOG4_RETVAL templog4_writeReg( uint8_t regAddr, uint16_t dataIn )
{
    uint8_t tempData[ 3 ];
    uint8_t numBytes;

    if ( (regAddr < 0x01) || ((regAddr > 0x04) && (regAddr != 0x22)) )
    {
        return _TEMPLOG4_ADDR_ERROR;
    }

    tempData[ 0 ] = regAddr;
    tempData[ 1 ] = dataIn >> 8;
    tempData[ 2 ] = dataIn;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, tempData, 3, END_MODE_STOP );

    return _TEMPLOG4_OK;
}

T_TEMPLOG4_RETVAL templog4_readReg( uint8_t regAddr, uint16_t *dataOut )
{
    uint8_t tempData[ 2 ];
    uint16_t retVal;
    uint8_t numBytes;

    if ((regAddr > 0x07) && (regAddr != 0x22))
    {
        return _TEMPLOG4_ADDR_ERROR;
    }

    tempData[ 0 ] = regAddr;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, tempData, 1, END_MODE_RESTART );
    hal_i2cRead( _slaveAddress, tempData, 2, END_MODE_STOP );

    retVal = tempData[ 0 ];
    retVal <<= 8;
    retVal |= tempData[ 1 ];
    *dataOut = retVal;

    return _TEMPLOG4_OK;
}

T_TEMPLOG4_RETVAL templog4_setAddrPtr( uint8_t regAddr )
{
    uint8_t registerAddr;

    if ((regAddr > 0x07) && (regAddr != 0x22))
    {
        return _TEMPLOG4_ADDR_ERROR;
    }
    
    registerAddr = regAddr;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, &registerAddr, 1, END_MODE_STOP );

    return _TEMPLOG4_OK;
}

void templog4_repeatedRead( uint16_t *dataOut )
{
    uint8_t tempData[ 2 ];
    uint16_t retVal;

    hal_i2cStart();
    hal_i2cRead( _slaveAddress, tempData, 2, END_MODE_STOP );

    retVal = tempData[ 0 ];
    retVal <<= 8;
    retVal |= tempData[ 1 ];
    *dataOut = retVal;
}

T_TEMPLOG4_RETVAL templog4_getTemp( uint8_t tempSel, T_TEMPLOG4_DEG *tempOut )
{
    T_TEMPLOG4_DEG tempRes;
    uint16_t temp;
    int16_t tempVal;
    T_TEMPLOG4_RETVAL limitStat;
    static uint8_t tempSelPrev = 0;

    if ((tempSel < 0x02) || (tempSel > 0x05))
    {
        return _TEMPLOG4_ADDR_ERROR;
    }

    if (tempSelPrev != tempSel)
    {
        templog4_setAddrPtr( tempSel );
        tempSelPrev = tempSel;
    }

    templog4_repeatedRead( &temp );

    limitStat = (temp & 0xE000) >> 8;

    if (temp & SIGN_BIT)
    {
        tempVal = temp | 0xE000;
    }
    else
    {
        tempVal = temp & 0x1FFF;
    }

    tempRes = tempVal * TEMP_RESOL;
    *tempOut = tempRes;

    return limitStat;
}

T_TEMPLOG4_RETVAL templog4_setTemp( uint8_t tempSel, T_TEMPLOG4_DEG tempIn )
{
    T_TEMPLOG4_DEG tempRes;
    int16_t tempVal;

    if ((tempSel < 0x02) || (tempSel > 0x04))
    {
        return _TEMPLOG4_ADDR_ERROR;
    }
    if ((tempIn < -40) || (tempIn > 125))
    {
        return _TEMPLOG4_TEMP_RANGE_ERROR;
    }

    tempRes = tempIn / TEMP_RESOL;
    tempVal = tempRes;
    tempVal &= 0x1FFC;

    templog4_writeReg( tempSel, tempVal );

    return _TEMPLOG4_OK;
}

T_TEMPLOG4_RETVAL templog4_checkAlarm( void )
{
    if (hal_gpio_intGet())
    {
        return _TEMPLOG4_ALARMING;
    }
    else
    {
        return _TEMPLOG4_OK;
    }
}

void templog4_waitConvDone( void )
{
    uint8_t timeCnt;

    for (timeCnt = 0; timeCnt < 13; timeCnt++)
    {
        Delay_10ms();
    }
}

void templog4_eepromByteWrite( uint8_t regAddr, uint8_t dataIn, uint8_t eepromMode )
{
    uint8_t tempData[ 2 ];
    uint8_t _slaveAddr;

    switch (eepromMode)
    {
        case _TEMPLOG4_EEPROM_WRITE :
        {
            _slaveAddr = _slaveEEPROM;
        break;
        }
        case _TEMPLOG4_SW_WRITE_PROTECT :
        {
            _slaveAddr = 0x31;
        break;
        }
        case _TEMPLOG4_CLEAR_WRITE_PROTECT :
        {
            _slaveAddr = 0x33;
        break;
        }
        default :
        {
            return;
        }
    }

    tempData[ 0 ] = regAddr;
    tempData[ 1 ] = dataIn;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddr, tempData, 2, END_MODE_STOP );
}

void templog4_eepromPageWrite( uint8_t regAddr, uint8_t *dataIn )
{
    uint8_t tempData[ 17 ];
    uint8_t cnt;

    tempData[ 0 ] = regAddr;

    for (cnt = 1; cnt <= 16; cnt++)
    {
        tempData[ cnt ] = *dataIn;
        dataIn++;
    }

    hal_i2cStart();
    hal_i2cWrite( _slaveEEPROM, tempData, 17, END_MODE_STOP );
}

void templog4_eepromCurrAddrRead( uint8_t *dataOut )
{
    hal_i2cStart();
    hal_i2cRead( _slaveEEPROM, dataOut, 1, END_MODE_STOP );
}

void templog4_eepromByteRead( uint8_t regAddr, uint8_t *dataOut )
{
    uint8_t registerAddr = regAddr;

    hal_i2cStart();
    hal_i2cWrite( _slaveEEPROM, &registerAddr, 1, END_MODE_RESTART );
    hal_i2cRead( _slaveEEPROM, dataOut, 1, END_MODE_STOP );
}

T_TEMPLOG4_RETVAL templog4_eepromSequentialRead( uint8_t regAddr, uint8_t *dataOut, uint16_t nBytes )
{
    uint8_t registerAddr = regAddr;

    if ((regAddr + nBytes) > _TEMPLOG4_EEPROM_SIZE)
    {
        return _TEMPLOG4_NBYTES_ERROR;
    }

    hal_i2cStart();
    hal_i2cWrite( _slaveEEPROM, &registerAddr, 1, END_MODE_RESTART );
    hal_i2cRead( _slaveEEPROM, dataOut, nBytes, END_MODE_STOP );

    return _TEMPLOG4_OK;
}

/* -------------------------------------------------------------------------- */
/*
  __templog4_driver.c

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */