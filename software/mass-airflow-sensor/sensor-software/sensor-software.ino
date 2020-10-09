#include <Wire.h>
#include "crc8.h"

// Test setup:
//   Arduino Nano pin asssignment:
//     I2C SDA:      A4
//     I2C SCL:      A5
//     Analog input: A0
//     Real-time supervision pin: A3
//     Interrupt supervision pin: A2

/* Mapping of the analog input pin, define alias here */
#define ANALOG_INPUT_PIN      (A0)

/* Mapping of the digital output pin for real-time supervision, define alias here */
#define RT_SUPERVISION_PIN    (A3)

/* Mapping of the digital output pin for interrupt supervision, define alias here */
#define INT_SUPERVISION_PIN   (A2)

//#define DEBUG
#ifdef DEBUG
    #define debugPrint    Serial.print
    #define debugPrintln  Serial.println
#else
    #define debugPrint    
    #define debugPrintln  
#endif

/* Definition of sensor loop interval and loop delay,
 * i.e. how often to read the ANALOG_INPUT_PIN in order to get a new measurement from sensor;
 * the value is given in milliseconds.
 * current setting: approx. every 10 ms (i.e. at a rate of 100 Hz).
 * Note: This is not exact as the UART serial output will take some of that time, 
 *       plus conversions and interrupt overhead, plus loop overhead.
 *       When DEBUG output is active, it takes about 2 ms per cycle.
 *       Caution: The interrupt overhead is not subtracted automatically.
 */
#define SENSOR_LOOP_INTERVAL (10u)
#ifdef DEBUG
    #define SENSOR_LOOP_DELAY (SENSOR_LOOP_INTERVAL - (2u))
#else
    #define SENSOR_LOOP_DELAY (SENSOR_LOOP_INTERVAL)
#endif

/* Definition of sensor modes, i.e. what values will be return upon Wire transmit request event */
enum eSensorMode { SENSOR_MODE_NONE = 0, SENSOR_MODE_MEASURE, SENSOR_MODE_SERIALNO };

/* define the address of the sensor on the I2C bus;
 * TODO/FIXME: decide address to be used via GPIO input pin during setup()
 */
uint8_t g_nDeviceAddress = 64;

/* Initialize global variables */
//volatile int g_nConversionValue = 0; /* digital value for sensor's analog input */
volatile eSensorMode g_eMode = SENSOR_MODE_NONE; /* sensor mode start with 'NONE' */

volatile int16_t g_nDiffPressure = 0;

void setup()
{
    Serial.begin(230400);         // start serial for debug output

    Wire.begin(g_nDeviceAddress); // join i2c bus with address <g_nDeviceAddress>
    Wire.onReceive(receiveEvent); // register event handler for Rx
    Wire.onRequest(transmitRequestEvent); // register event handler for Tx request

#ifdef RT_SUPERVISION_PIN
    pinMode(RT_SUPERVISION_PIN, OUTPUT);
#endif
#ifdef INT_SUPERVISION_PIN
    pinMode(INT_SUPERVISION_PIN, OUTPUT);
#endif

#ifdef DEBUG
    Serial.println("Debug mode active.");
#else
    Serial.println("Debug mode inactive, no more output on serial.");
#endif
}

void loop()
{
    static int nConversionValue = 0;

#ifdef RT_SUPERVISION_PIN
    digitalWrite(RT_SUPERVISION_PIN, HIGH);
#endif
    
    nConversionValue = analogRead(ANALOG_INPUT_PIN);

    if( SENSOR_MODE_MEASURE == g_eMode )
    {
        debugPrint("[*] "); /* in measuring mode */
    }
    else
    {
        debugPrint("[ ] "); /* not in measuring mode */
    }

    // Debug output the sensor's reading (conversion valie)
    debugPrint("Sensor value: ");

    debugPrint(nConversionValue); // raw reading
    debugPrint(" counts, ");

    debugPrint( countsToMillivolts(nConversionValue) );
    debugPrint(" mV, ");

    float fPressure = millivoltsToMillimetersWater( countsToMillivolts( nConversionValue ) );
    debugPrint( fPressure );
    debugPrint(" mmH2O, ");

    g_nDiffPressure = ((int16_t)fPressure) << 2; /* the two least-significant bits are always zero */
    debugPrint( g_nDiffPressure );
    debugPrintln(" mmH2O*4");

    /* TODO/FIXME: add conversion from difference pressure to volume flow */

#ifdef RT_SUPERVISION_PIN
    digitalWrite(RT_SUPERVISION_PIN, LOW);
#endif

    /* Wait until next cyclic measurement is made */
    delay(SENSOR_LOOP_DELAY);
}

crc_t calcCrc(const unsigned char *pData, size_t nDataLen)
{
    crc_t nCrc = crc_init();
    nCrc = crc_update(nCrc, pData, nDataLen);
    nCrc = crc_finalize(nCrc);

    return nCrc;
}

/* Parse received command and switch sensor's state */
void receiveEvent(int nBytes)
{
    char sHexBuf[11]; /* string buffer for debug output via UART */
    uint8_t cCmdBytes[2]; /* command buffer (a command is 16 bit wide, or 2 bytes) */
    int nRxCount = 0;

#ifdef INT_SUPERVISION_PIN
    digitalWrite(INT_SUPERVISION_PIN, HIGH);
#endif

    debugPrint("Rx (");
    debugPrint(nBytes);
    debugPrint(" bytes): ");
    while(0 < Wire.available())   // loop through all but the last
    {
        uint8_t cRxByte = Wire.read(); /* receive byte as a character */

        /* store first two bytes in command buffer */
        if( nRxCount < 2 )
        {
            cCmdBytes[nRxCount] = cRxByte;
        }
        
        nRxCount++;

        /* print every byte as two hex digits with prefix '0x', NULL-terminate the string */
        sprintf(sHexBuf, "0x%02X ", cRxByte);
        sHexBuf[5] = 0;
        debugPrint(sHexBuf);
    }
    debugPrintln("");

    if( nRxCount < 2 )
    {
#ifdef INT_SUPERVISION_PIN
        digitalWrite(INT_SUPERVISION_PIN, LOW);
#endif
        return;
    }

    if( nRxCount != nBytes )
    {
        debugPrint("[DEBUG] Message length does not match (Rx count: ");
        debugPrint(nRxCount);
        debugPrint("; Message length: ");
        debugPrint(nBytes);
        debugPrintln(")");
        g_eMode = SENSOR_MODE_NONE;
#ifdef INT_SUPERVISION_PIN
        digitalWrite(INT_SUPERVISION_PIN, LOW);
#endif
        return;
    }

    if( 0x10 == cCmdBytes[0] && 0x00 == cCmdBytes[1] )
    {
        debugPrintln("Rx'ed 'start measurement' command");
        g_eMode = SENSOR_MODE_MEASURE;
    }
    else if( 0x31 == cCmdBytes[0] && 0xAE == cCmdBytes[1] )
    {
        debugPrintln("Rx'ed 'read serial number' command");
        g_eMode = SENSOR_MODE_SERIALNO;
    }
    else
    {
        debugPrintln("Rx'ed unknown command: ");
        sprintf(sHexBuf, "0x%02X 0x%02X ", cCmdBytes[0], cCmdBytes[1]);
        sHexBuf[10] = 0;
        debugPrint(sHexBuf);
        
        debugPrintln("(ignored)");
    }

#ifdef INT_SUPERVISION_PIN
        digitalWrite(INT_SUPERVISION_PIN, LOW);
#endif
}

/* Transmit data dependent on current sensor state */
void transmitRequestEvent(void)
{
#ifdef INT_SUPERVISION_PIN
    digitalWrite(INT_SUPERVISION_PIN, HIGH);
#endif

    debugPrintln("Tx request");

    switch( g_eMode )
    {
        case SENSOR_MODE_MEASURE:
            /* TODO/FIXME: add true measurement data and add CRC */
            debugPrintln("Tx measurement");
            char sMeas[2];
            //sMeas[0] = 'F';
            //sMeas[1] = 'L';
            sMeas[0] = (uint8_t)( (g_nDiffPressure & 0xFF00) >> 8 );
            sMeas[1] = (uint8_t)(g_nDiffPressure & 0xFF);
            Wire.write((char)sMeas[0]);
            Wire.write((char)sMeas[1]);
            Wire.write((char)calcCrc(&sMeas[0], 2) );
            break;
        case SENSOR_MODE_SERIALNO:
            char sSerNo[4];
            sSerNo[0] = 'S';
            sSerNo[1] = 'R';
            sSerNo[2] = 'N';
            sSerNo[3] = 'O';
            debugPrintln("Tx serialno");
            Wire.write((char)sSerNo[0]);
            Wire.write((char)sSerNo[1]);
            Wire.write((char)calcCrc(&sSerNo[0], 2) );
            Wire.write((char)sSerNo[2]);
            Wire.write((char)sSerNo[3]);
            Wire.write((char)calcCrc(&sSerNo[2], 2) );
            break;
        case SENSOR_MODE_NONE:
        default:
            debugPrintln("Tx nothing (controller will read 0xFF)");
            break;
    }

#ifdef INT_SUPERVISION_PIN
        digitalWrite(INT_SUPERVISION_PIN, LOW);
#endif
}

float countsToMillivolts(int nCounts)
{
    // For the Arduino Uno, its a 10 bit unsgined value (i.e. a range of 0..1023).
    // Its measurement range is from 0..5 volts.
    // This yields a resolution between readings of: 5 volts / 1024 units or approx. 4.9 mV per LSB
    return nCounts * 4.882813f;
}

float millivoltsToMillimetersWater(float fMillivolts)
{
    // for now, assume a linear transfer function of the sensor,
    // e.g. V_out = 0.6 V/kPa * dP + 0.6 V
    //         dP = (V_out - 0.6 V) / 0.6 V/kPa 
    //            = (V_out - 0.6 V) * 1.667 kPa/V
    //            = (V_out - 600 mV) * (1.667*100 mmH2O) / 1000 mV // 1 kPa ~ approx. 100 mmH2O

    float fPressure = (fMillivolts - 600.0f) * 0.1666666667f;

    // sanity check: no negative pressure value
    fPressure = (fPressure < 0.0f) ? 0.0f : fPressure;
    
    return fPressure;
}

