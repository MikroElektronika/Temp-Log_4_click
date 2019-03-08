![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

---

# Temp_Log_4 Click

- **CIC Prefix**  : TEMPLOG4
- **Author**      : Nemanja Medakovic
- **Verison**     : 1.0.0
- **Date**        : Dec 2018.

---

### Software Support

We provide a library for the Temp_Log_4 Click on our [LibStock](https://libstock.mikroe.com/projects/view/2684/temp-log-4-click) 
page, as well as a demo application (example), developed using MikroElektronika 
[compilers](http://shop.mikroe.com/compilers). The demo can run on all the main 
MikroElektronika [development boards](http://shop.mikroe.com/development-boards).

**Library Description**

The library performs a control of the Temp-Log 4 Click board.
There are a functions that offer a choice to measure the ambient temperature and set a temperature limit values to generate alarm.
The library also can save a desired configurations or temperature values to the EEPROM, which has size of 256 bytes. 
For more details check documentation.

Key functions :

- ``` T_TEMPLOG4_RETVAL templog4_writeReg( uint8_t regAddr, uint16_t dataIn ) ``` - Function writes a 16bit data to the desired register.
- ``` void templog4_repeatedRead( uint16_t *dataOut ) ``` - Function reads a 16bit data from the register on which the internal address pointer was last set.
- ``` T_TEMPLOG4_RETVAL templog4_getTemp( uint8_t tempSel, T_TEMPLOG4_DEG *tempOut ) ``` - Function gets a temperature value from the desired temperature register calculated to the Celsius degrees.
- ``` T_TEMPLOG4_RETVAL templog4_setTemp( uint8_t tempSel, T_TEMPLOG4_DEG tempIn ) ``` - Function sets a desired temperature register on the desired value calculated to the Celsius degrees.

**Examples Description**

The application is composed of three sections :

- System Initialization - Initializes peripherals and pins.
- Application Initialization - Initializes I2C interface and performs a device configuration for properly working.
  Also sets the temperature limit to the desired values.
- Application Task - (code snippet) - First ensures that the minimum conversion time is met, and then reads the
  ambient temperature calculated to the Celsius degrees.
  Also checks the limit status and shows a message when some limit condition is met.
Note : The temperature range that can be measured or written is from -40 to +125 Celsius degrees.
The resolution of ambient temperature measurement is 11bit, or 0.125 Celsius degrees.
The limit temperature resolution is always a 10bit, or 0.25 Celsius degrees.
If user wants to enable the EEPROM Write Protection, the A0 pin on the device must be set to the high voltage level.
This high voltage level for EEPROM Write Protection must be in range from 7V to 10V, in other cases can be up to VDD.


```.c
void applicationTask()
{
    templog4_waitConvDone();

    retStatus = templog4_getTemp( _TEMPLOG4_TEMP_AMBIENT_REG, &temperature );

    FloatToStr( temperature, text );
    floatCut();
    mikrobus_logWrite( "**  Ambient temperature is : ", _LOG_TEXT );
    mikrobus_logWrite( text, _LOG_TEXT );
    logUnit();
    mikrobus_logWrite( "  **", _LOG_LINE );

    checkLimitStatus();

    Delay_ms( 300 );
}
```

Additional Functions :

- floatCut - Makes that float values be rounded on two decimal places.
- logUnit - Writes a Celsius degrees symbol on uart terminal.
- checkLimitStatus - Checks the limit status for each temperature reading cycle and writes a message on uart terminal
  when some limit condition is met.

The full application code, and ready to use projects can be found on our 
[LibStock](https://libstock.mikroe.com/projects/view/2684/temp-log-4-click) page.

Other mikroE Libraries used in the example:

- Conversions
- I2C
- UART

**Additional notes and informations**

Depending on the development board you are using, you may need 
[USB UART click](http://shop.mikroe.com/usb-uart-click), 
[USB UART 2 Click](http://shop.mikroe.com/usb-uart-2-click) or 
[RS232 Click](http://shop.mikroe.com/rs232-click) to connect to your PC, for 
development systems with no UART to USB interface available on the board. The 
terminal available in all Mikroelektronika 
[compilers](http://shop.mikroe.com/compilers), or any other terminal application 
of your choice, can be used to read the message.

---
---
