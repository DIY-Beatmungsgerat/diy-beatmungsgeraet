#include <Wire.h>

// Test setup:
//   Arduino Nano pin asssignment:
//     I2C SDA:      A4
//     I2C SCL:      A5
//     Analog input: A0

/* Mapping of the analog input pin, define alias here */
#define ANALOG_INPUT_PIN  (A0)

/* Definition of loop delay,
 * i.e. how often to read the ANALOG_INPUT_PIN in order to get a new measurement from sensor;
 * the value is given in milliseconds.
 * current setting: approx. every 250 ms (i.e. at a rate of 4 Hz).
 * Note: This is not exact as the UART serial output will take some of that time, 
 *       plus conversions and interrupt overhead, plus loop overhead.
 */
#define SENSOR_LOOP_DELAY (250u)

/* Definition of sensor modes, i.e. what values will be return upon Wire transmit request event */
enum eSensorMode { SENSOR_MODE_NONE = 0, SENSOR_MODE_MEASURE, SENSOR_MODE_SERIALNO };

/* define the address of the sensor on the I2C bus;
 * TODO/FIXME: decide address to be used via GPIO input pin during setup()
 */
uint8_t g_nDeviceAddress = 64;

/* Initialize global variables */
volatile int g_nConversionValue = 0; /* digital value for sensor's analog input */
volatile eSensorMode g_eMode = SENSOR_MODE_NONE; /* sensor mode start with 'NONE' */


void setup()
{
    Wire.begin(g_nDeviceAddress); // join i2c bus with address <g_nDeviceAddress>
    Wire.onReceive(receiveEvent); // register event handler for Rx
    Wire.onRequest(transmitRequestEvent); // register event handler for Tx request
    Serial.begin(230400);         // start serial for debug output
}

void loop()
{
    g_nConversionValue = analogRead(ANALOG_INPUT_PIN);

    if( SENSOR_MODE_MEASURE == g_eMode )
    {
        Serial.print("[*] "); /* in measuring mode */
    }
    else
    {
        Serial.print("[ ] "); /* not in measuring mode */
    }

    // Debug output the sensor's reading (conversion valie)
    Serial.print("Sensor value: ");

    Serial.print(g_nConversionValue); // raw reading
    Serial.print(" counts, ");

    Serial.print( countsToMillivolts(g_nConversionValue) );
    Serial.print(" mV, ");

    Serial.print( millivoltsToMillimetersWater( countsToMillivolts( g_nConversionValue ) ) );
    Serial.println(" mmH2O");

    /* Wait until next cyclic measurement is made */
    delay(SENSOR_LOOP_DELAY);
}

/* Parse received command and switch sensor's state */
void receiveEvent(int nBytes)
{
    char sHexBuf[11]; /* string buffer for debug output via UART */
    uint8_t cCmdBytes[2]; /* command buffer (a command is 16 bit wide, or 2 bytes) */
    int nRxCount = 0;

    Serial.print("Rx (");
    Serial.print(nBytes);
    Serial.print(" bytes): ");
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
        Serial.print(sHexBuf);
    }
    Serial.println();

    if( nRxCount < 2 )
    {
        return;
    }

    if( nRxCount != nBytes )
    {
        Serial.print("[DEBUG] Message length does not match (Rx count: ");
        Serial.print(nRxCount);
        Serial.print("; Message length: ");
        Serial.print(nBytes);
        Serial.println(")");
        g_eMode = SENSOR_MODE_NONE;
        return;
    }

    if( 0x10 == cCmdBytes[0] && 0x00 == cCmdBytes[1] )
    {
        Serial.println("Rx'ed 'start measurement' command");
        g_eMode = SENSOR_MODE_MEASURE;
    }
    else if( 0x31 == cCmdBytes[0] && 0xAE == cCmdBytes[1] )
    {
        Serial.println("Rx'ed 'read serial number' command");
        g_eMode = SENSOR_MODE_SERIALNO;
    }
    else
    {
        Serial.println("Rx'ed unknown command: ");
        sprintf(sHexBuf, "0x%02X 0x%02X ", cCmdBytes[0], cCmdBytes[1]);
        sHexBuf[10] = 0;
        Serial.print(sHexBuf);
        
        Serial.println("(ignored)");
    }
}

/* Transmit data dependent on current sensor state */
void transmitRequestEvent(void)
{
    Serial.println("Tx request");

    switch( g_eMode )
    {
        case SENSOR_MODE_MEASURE:
            /* TODO/FIXME: add true measurement data and add CRC */
            Serial.println("Tx measurement");
            Wire.write((char)'F');
            Wire.write((char)'L');
            Wire.write((char)'-');
            break;
        case SENSOR_MODE_SERIALNO:
            /* TODO/FIXME: add correct CRC for fake serial no */
            Serial.println("Tx serialno");
            Wire.write((char)'S');
            Wire.write((char)'R');
            Wire.write((char)'-');
            Wire.write((char)'N');
            Wire.write((char)'O');
            Wire.write((char)'-');
            break;
        case SENSOR_MODE_NONE:
        default:
            Serial.println("Tx nothing (controller will read 0xFF)");
            break;
    }
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

