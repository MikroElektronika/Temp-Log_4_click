/*
    __templog4_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __templog4_driver.h
@brief    Temp_Log_4 Driver
@mainpage Temp_Log_4 Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   TEMPLOG4
@brief      Temp_Log_4 Click Driver
@{

| Global Library Prefix | **TEMPLOG4** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Dec 2018.**      |
| Developer             | **Nemanja Medakovic**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _TEMPLOG4_H_
#define _TEMPLOG4_H_

/** 
 * @macro T_TEMPLOG4_P
 * @brief Driver Abstract type 
 */
#define T_TEMPLOG4_P    const uint8_t*
#define T_TEMPLOG4_RETVAL     uint8_t
#define T_TEMPLOG4_DEG        float

#define TEMP_RESOL  0.0625
#define SIGN_BIT    0x1000

/** @defgroup TEMPLOG4_COMPILE Compilation Config */              /** @{ */

//  #define   __TEMPLOG4_DRV_SPI__                            /**<     @macro __TEMPLOG4_DRV_SPI__  @brief SPI driver selector */
   #define   __TEMPLOG4_DRV_I2C__                            /**<     @macro __TEMPLOG4_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __TEMPLOG4_DRV_UART__                           /**<     @macro __TEMPLOG4_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup TEMPLOG4_VAR Variables */                           /** @{ */

/** Device Address */
extern const uint8_t _TEMPLOG4_SLAVE_ADDR_A0_H              ;
extern const uint8_t _TEMPLOG4_SLAVE_ADDR_A0_L              ;

/** Register Address */
extern const uint8_t _TEMPLOG4_CAPABILITY_REG               ;
extern const uint8_t _TEMPLOG4_CONFIG_REG                   ;
extern const uint8_t _TEMPLOG4_TEMP_UPPER_REG               ;
extern const uint8_t _TEMPLOG4_TEMP_LOWER_REG               ;
extern const uint8_t _TEMPLOG4_TEMP_CRITICAL_REG            ;
extern const uint8_t _TEMPLOG4_TEMP_AMBIENT_REG             ;
extern const uint8_t _TEMPLOG4_MANUFACT_ID_REG              ;
extern const uint8_t _TEMPLOG4_DEVICE_ID_REG                ;
extern const uint8_t _TEMPLOG4_SMBUS_REG                    ;

/** Status mask for the Capability register */
extern const uint8_t _TEMPLOG4_EVENT_SHDN_STATUS_MASK       ;
extern const uint8_t _TEMPLOG4_BUS_TIMEOUT_STATUS_MASK      ;
extern const uint8_t _TEMPLOG4_HIGH_VOLT_INPUT_STATUS_MASK  ;
extern const uint8_t _TEMPLOG4_RESOLUTION_STATUS_MASK       ;
extern const uint8_t _TEMPLOG4_MEAS_RANGE_STATUS_MASK       ;
extern const uint8_t _TEMPLOG4_ACCURACY_STATUS_MASK         ;
extern const uint8_t _TEMPLOG4_ALARM_STATUS_MASK            ;

/** Settings for the Config register */
extern const uint16_t _TEMPLOG4_TLIMIT_HYST_0_DEG           ;
extern const uint16_t _TEMPLOG4_TLIMIT_HYST_ONE_HALF_DEG    ;
extern const uint16_t _TEMPLOG4_TLIMIT_HYST_3_DEG           ;
extern const uint16_t _TEMPLOG4_TLIMIT_HYST_6_DEG           ;
extern const uint16_t _TEMPLOG4_CONT_CONV_MODE              ;
extern const uint16_t _TEMPLOG4_SHUTDOWN_MODE               ;
extern const uint16_t _TEMPLOG4_TCRIT_LOCKED                ;
extern const uint16_t _TEMPLOG4_TUPPER_TLOWER_LOCKED        ;
extern const uint16_t _TEMPLOG4_INT_CLEAR                   ;
extern const uint16_t _TEMPLOG4_EVENT_OUTPUT_STATUS_MASK    ;
extern const uint16_t _TEMPLOG4_EVENT_OUTPUT_EN             ;
extern const uint16_t _TEMPLOG4_EVENT_ALL_TLIMIT            ;
extern const uint16_t _TEMPLOG4_EVENT_TCRIT_ONLY            ;
extern const uint16_t _TEMPLOG4_EVENT_POL_ACT_LOW           ;
extern const uint16_t _TEMPLOG4_EVENT_POL_ACT_HIGH          ;
extern const uint16_t _TEMPLOG4_EVENT_COMPARATOR_MODE       ;
extern const uint16_t _TEMPLOG4_EVENT_INTERRUPT_MODE        ;

/** Limit and function response possible status bytes */
extern const uint8_t _TEMPLOG4_TCRIT_DETECT                 ;
extern const uint8_t _TEMPLOG4_TUPPER_DETECT                ;
extern const uint8_t _TEMPLOG4_TLOWER_DETECT                ;
extern const uint8_t _TEMPLOG4_NBYTES_ERROR                 ;
extern const uint8_t _TEMPLOG4_TEMP_RANGE_ERROR             ;
extern const uint8_t _TEMPLOG4_ADDR_ERROR                   ;
extern const uint8_t _TEMPLOG4_ALARMING                     ;
extern const uint8_t _TEMPLOG4_OK                           ;

/** Settings for the SMBus register */
extern const uint8_t _TEMPLOG4_SMBUS_TIMEOUT_DIS            ;
extern const uint8_t _TEMPLOG4_SMBUS_TIMEOUT_SHTDN_EN       ;
extern const uint8_t _TEMPLOG4_SMBUS_ARA_DIS                ;

/** Settings for the EEPROM writing */
extern const uint8_t _TEMPLOG4_EEPROM_WRITE                 ;
extern const uint8_t _TEMPLOG4_SW_WRITE_PROTECT             ;
extern const uint8_t _TEMPLOG4_CLEAR_WRITE_PROTECT          ;

/** Maximal EEPROM size in bytes */
extern const uint16_t _TEMPLOG4_EEPROM_SIZE                 ;

extern const uint16_t _TEMPLOG4_MANUFACT_ID                 ;
extern const uint16_t _TEMPLOG4_DEVICE_ID                   ;
extern const uint8_t _TEMPLOG4_DUMMY_BYTE                   ;

                                                                       /** @} */
/** @defgroup TEMPLOG4_TYPES Types */                             /** @{ */



                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup TEMPLOG4_INIT Driver Initialization */              /** @{ */

#ifdef   __TEMPLOG4_DRV_SPI__
void templog4_spiDriverInit(T_TEMPLOG4_P gpioObj, T_TEMPLOG4_P spiObj);
#endif
#ifdef   __TEMPLOG4_DRV_I2C__
void templog4_i2cDriverInit(T_TEMPLOG4_P gpioObj, T_TEMPLOG4_P i2cObj, uint8_t slave);
#endif
#ifdef   __TEMPLOG4_DRV_UART__
void templog4_uartDriverInit(T_TEMPLOG4_P gpioObj, T_TEMPLOG4_P uartObj);
#endif

                                                                       /** @} */
/** @defgroup TEMPLOG4_FUNC Driver Functions */                   /** @{ */

/**
 * @brief Generic Write function
 *
 * @param[in] regAddr  Address where data be written
 * @param[in] dataIn  16bit data to be written
 *
 * @returns 0 - OK, 2 - Wrong address
 *
 * Function writes a 16bit data to the desired register.
 */
T_TEMPLOG4_RETVAL templog4_writeReg( uint8_t regAddr, uint16_t dataIn );

/**
 * @brief Generic Read function
 *
 * @param[in] regAddr  Address which from data be read
 * @param[out] dataOut  Memory where 16bit data be placed
 *
 * @returns 0 - OK, 2 - Wrong address
 *
 * Function reads a 16bit data from the desired register.
 */
T_TEMPLOG4_RETVAL templog4_readReg( uint8_t regAddr, uint16_t *dataOut );

/**
 * @brief Set Address Pointer function
 *
 * @param[in] regAddr  Address on which the internal address pointer be set
 *
 * @returns 0 - OK, 2 - Wrong address
 *
 * Function sets the internal address pointer on the desired register address.
 */
T_TEMPLOG4_RETVAL templog4_setAddrPtr( uint8_t regAddr );

/**
 * @brief Repeated Read function
 *
 * @param[out] dataOut  Memory where 16bit data be placed
 *
 * Function reads a 16bit data from the register on which the internal address pointer was last set.
 */
void templog4_repeatedRead( uint16_t *dataOut );

/**
 * @brief Get Temperature function
 *
 * @param[in] tempSel  Address of the desired temperature register
 * @param[out] tempOut  Returns a temperature value calculated to the Celsius degrees
 *
 * @returns 0 - OK, 2 - Wrong address
 *
 * Function gets a temperature value from the desired temperature register calculated to the Celsius degrees.
 */
T_TEMPLOG4_RETVAL templog4_getTemp( uint8_t tempSel, T_TEMPLOG4_DEG *tempOut );

/**
 * @brief Set Temperature function
 *
 * @param[in] tempSel  Address of the desired temperature register
 * @param[in] tempIn  Temperature value to be written calculated to the Celsius degrees
 *
 * @returns 0 - OK, 2 - Wrong address, 3 - Temperature value is out of range
 *
 * Function sets a desired temperature register on the desired value calculated to the Celsius degrees.
 */
T_TEMPLOG4_RETVAL templog4_setTemp( uint8_t tempSel, T_TEMPLOG4_DEG tempIn );

/**
 * @brief Alarm-Event Check function
 *
 * @returns 0 - OK, 1 - Alarming, when alarm (event) pin polarity is set to active high
 *
 * Function checks the alarm (event) pin state.
 */
T_TEMPLOG4_RETVAL templog4_checkAlarm( void );

/**
 * @brief Conversion Time function
 *
 * Function ensures that the minimum conversion time is met.
 */
void templog4_waitConvDone( void );

/**
 * @brief EEPROM Single Write function
 *
 * @param[in] regAddr  Memory address where one byte data be written
 * @param[in] dataIn  Data byte to be written
 * @param[in] eepromMode  0 - EEPROM Write, 1 - SW Write Protection, 2 - Clear Write Protection
 *
 * Function writes a one byte data to the EEPROM including/excluding a write protection.
 */
void templog4_eepromByteWrite( uint8_t regAddr, uint8_t dataIn, uint8_t eepromMode );

/**
 * @brief EEPROM Page Write function
 *
 * @param[in] regAddr  Memory start address which from a writing cycle be started
 * @param[in] dataIn  Data to be written, 16 bytes of data
 *
 * Function writes a 16 bytes of data to the selected EEPROM page.
 */
void templog4_eepromPageWrite( uint8_t regAddr, uint8_t *dataIn );

/**
 * @brief EEPROM Current Address Read function
 *
 * @param[out] dataOut  Returns a value from the address of the last word accessed internally incremented by 1
 *
 * Function returns a value if the address of the last word accessed internally incremented by 1.
 */
void templog4_eepromCurrAddrRead( uint8_t *dataOut );

/**
 * @brief EEPROM Single Read function
 *
 * @param[in] regAddr  Memory address which from data byte be read
 * @param[out] dataOut  Memory where data byte be placed
 *
 * Function reads a one byte data from the EEPROM.
 */
void templog4_eepromByteRead( uint8_t regAddr, uint8_t *dataOut );

/**
 * @brief EEPROM Sequential Read function
 *
 * @param[in] regAddr  Memory start address which from a reading cycle be started
 * @param[out] dataOut  Memory where data be placed
 * @param[in] nBytes  Number of bytes to be read
 *
 * @returns 0 - OK, 4 - Number of bytes is out of range
 *
 * Function reads a desired number of bytes from the EEPROM.
 * @note Sum of the memory address and number of bytes must not be higher than 256.
 */
T_TEMPLOG4_RETVAL templog4_eepromSequentialRead( uint8_t regAddr, uint8_t *dataOut, uint16_t nBytes );

                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_Temp_Log_4_STM.c
    @example Click_Temp_Log_4_TIVA.c
    @example Click_Temp_Log_4_CEC.c
    @example Click_Temp_Log_4_KINETIS.c
    @example Click_Temp_Log_4_MSP.c
    @example Click_Temp_Log_4_PIC.c
    @example Click_Temp_Log_4_PIC32.c
    @example Click_Temp_Log_4_DSPIC.c
    @example Click_Temp_Log_4_AVR.c
    @example Click_Temp_Log_4_FT90x.c
    @example Click_Temp_Log_4_STM.mbas
    @example Click_Temp_Log_4_TIVA.mbas
    @example Click_Temp_Log_4_CEC.mbas
    @example Click_Temp_Log_4_KINETIS.mbas
    @example Click_Temp_Log_4_MSP.mbas
    @example Click_Temp_Log_4_PIC.mbas
    @example Click_Temp_Log_4_PIC32.mbas
    @example Click_Temp_Log_4_DSPIC.mbas
    @example Click_Temp_Log_4_AVR.mbas
    @example Click_Temp_Log_4_FT90x.mbas
    @example Click_Temp_Log_4_STM.mpas
    @example Click_Temp_Log_4_TIVA.mpas
    @example Click_Temp_Log_4_CEC.mpas
    @example Click_Temp_Log_4_KINETIS.mpas
    @example Click_Temp_Log_4_MSP.mpas
    @example Click_Temp_Log_4_PIC.mpas
    @example Click_Temp_Log_4_PIC32.mpas
    @example Click_Temp_Log_4_DSPIC.mpas
    @example Click_Temp_Log_4_AVR.mpas
    @example Click_Temp_Log_4_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __templog4_driver.h

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