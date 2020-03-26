#include <Wire.h>

// Test setup:
//   Arduino Nano pin asssignment:
//     I2C SDA:      A4
//     I2C SCL:      A5
//     Analog input: A0

#define ANALOG_INPUT_PIN  (A0)

/* define the address of the sensor on the I2C bus */
uint8_t g_nDeviceAddress = 64;

int g_nConversionValue = 0; // digital value for sensor's analog input

float countsToMillivolts(int nCounts);
float millivoltsToMillimetersWater(float fMillivolts);

void setup()
{
    // TODO: decide address to be used via GPIO input pin
    Wire.begin(g_nDeviceAddress); // join i2c bus with address <g_nDeviceAddress>
    Wire.onReceive(receiveEvent); // register event handler
    Serial.begin(9600);           // start serial for debug output
}

void loop()
{
    // read an analog input to get measurement from sensor
    // approx. every 250 ms (i.e. at a rate of 4 Hz)
    // (the serial output will take some of that time + loop, conversions and interrupt overhead)
    delay(250);
    g_nConversionValue = analogRead(ANALOG_INPUT_PIN);

    // Debug output the sensor's reading (conversion valie)
    Serial.print("Sensor value: ");

    Serial.print(g_nConversionValue); // raw reading
    Serial.print(" counts, ");

    Serial.print( countsToMillivolts(g_nConversionValue) );
    Serial.print(" mV, ");

    Serial.print( millivoltsToMillimetersWater( countsToMillivolts( g_nConversionValue ) ) );
    Serial.println(" mmH2O");
}

void receiveEvent(int nBytes)
{
    char sHexBuf[6];

    Serial.print("Rx (");
    Serial.print(nBytes);
    Serial.print(" bytes):");
    while(0 < Wire.available())   // loop through all but the last
    {
        char cRxByte = Wire.read(); // receive byte as a character
        sprintf(sHexBuf, "0x%02X ", cRxByte);
        //Serial.print(cRxByte, HEX);
        sHexBuf[5] = 0;
        Serial.print(sHexBuf);
    }
    Serial.println();           // print a line break
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

