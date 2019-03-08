/*
Example for Temp_Log_4 Click

    Date          : Dec 2018.
    Author        : Nemanja Medakovic

Test configuration MSP :
    
    MCU              : MSP432
    Dev. Board       : Clicker 2 for MSP432
    ARM Compiler ver : v6.0.0.0

---

Description :

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

Additional Functions :

- floatCut - Makes that float values be rounded on two decimal places.
- logUnit - Writes a Celsius degrees symbol on uart terminal.
- checkLimitStatus - Checks the limit status for each temperature reading cycle and writes a message on uart terminal
  when some limit condition is met.

*/

#include "Click_Temp_Log_4_types.h"
#include "Click_Temp_Log_4_config.h"

T_TEMPLOG4_RETVAL retStatus;
T_TEMPLOG4_DEG temperature;
char text[ 50 ];

void floatCut()
{
    uint8_t count;
    uint8_t conCnt = 0;
    uint8_t conVar = 0;

    for (count = 0; count < 50; count++)
    {
        if (text[ count ] == '.')
        {
            conVar = 1;
        }
        if (conVar == 1)
        {
            conCnt++;
        }
        if (conCnt > 3)
        {
            if ((text[ count ] == 'e') || (conVar == 2))
            {
                text[ count - (conCnt - 4) ] = text[ count ];
                text[ count ] = 0;
                conVar = 2;
            }
            else
            {
                text[ count ] = 0;
            }
        }
    }
}

void logUnit()
{
    text[ 0 ] = ' ';
    text[ 1 ] = 176;
    text[ 2 ] = 'C';
    text[ 3 ] = 0;

    mikrobus_logWrite( text, _LOG_TEXT );
}

void checkLimitStatus()
{
    if ((retStatus & _TEMPLOG4_TCRIT_DETECT) != _TEMPLOG4_OK)
    {
        mikrobus_logWrite( "**  Critical temperature detection!  **", _LOG_LINE );
    }
    if ((retStatus & _TEMPLOG4_TUPPER_DETECT) != _TEMPLOG4_OK)
    {
        mikrobus_logWrite( "**  Ambient temperature is higher than upper limit temperature!  **", _LOG_LINE );
    }
    else if ((retStatus & _TEMPLOG4_TLOWER_DETECT) != _TEMPLOG4_OK)
    {
        mikrobus_logWrite( "**  Ambient temperature is lower than lower limit temperature!  **", _LOG_LINE );
    }
}

void systemInit()
{
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_INT_PIN, _GPIO_INPUT );

    mikrobus_i2cInit( _MIKROBUS1, &_TEMPLOG4_I2C_CFG[0] );

    mikrobus_logInit( _MIKROBUS2, 9600 );
    mikrobus_logWrite( "*** Initializing... ***", _LOG_LINE );

    Delay_ms( 100 );
}

void applicationInit()
{
    templog4_i2cDriverInit( (T_TEMPLOG4_P)&_MIKROBUS1_GPIO, (T_TEMPLOG4_P)&_MIKROBUS1_I2C, _TEMPLOG4_SLAVE_ADDR_A0_L );
    Delay_ms( 500 );

    templog4_writeReg( _TEMPLOG4_CONFIG_REG, _TEMPLOG4_TLIMIT_HYST_0_DEG | _TEMPLOG4_CONT_CONV_MODE | _TEMPLOG4_EVENT_OUTPUT_EN | _TEMPLOG4_EVENT_TCRIT_ONLY | _TEMPLOG4_EVENT_POL_ACT_HIGH | _TEMPLOG4_EVENT_COMPARATOR_MODE );
    templog4_setTemp( _TEMPLOG4_TEMP_CRITICAL_REG, 28.5 );
    templog4_setTemp( _TEMPLOG4_TEMP_UPPER_REG, 40.250 );
    templog4_setTemp( _TEMPLOG4_TEMP_LOWER_REG, -5 );
    Delay_ms( 200 );

    mikrobus_logWrite( "** Temp-Log 4 is initialized **", _LOG_LINE );
    mikrobus_logWrite( "", _LOG_LINE );
}

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

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
        applicationTask();
    }
}
