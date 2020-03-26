#include <Wire.h>

// Test setup:
//   Arduino Leonardo:
//     pin asssignment 2 (SDA), 3 (SCL)

/* define the address of the sensor on the I2C bus */
uint8_t g_nDeviceAddress = 64;

enum retval { SENSOR_SUCCESS = 0, SENSOR_FAIL };

void setup()
{
    Wire.begin();
    Serial.begin(9600); // start serial for debug output
}

// * (public)  retval readMeasurement(int16_t* pnVal) =
// * (private) retval sendStartMeasurementCmd(void) +
// * (private) retval readMeasurementValue(int16_t* nVal)
retval readMeasurement(int16_t* pnVal);

// * (public)  retval readSerialNumber(int32_t* pnSerialNo) =
// * (private) retval sendReadSerialNumberCmd(void) +
// * (private) retval readSerialNumberValue(int32_t* pnSerialNo)
retval readSerialNumber(int32_t* pnSerialNo);

void loop()
{
    uint16_t nVal = 0;
    uint32_t nSerialNo = 0xBADC0FFE;

    if( SENSOR_SUCCESS == readMeasurement(&nVal) )
    {
        Serial.print("Read measurement; raw: ");
        Serial.print(nVal, HEX);
        Serial.println();
    }
    else
    {
        Serial.println("Failed to read measurement");
    }

    delay(1000);

    if( SENSOR_SUCCESS == readSerialNumber(&nSerialNo) )
    {
        Serial.print("Read serial number: ");
        Serial.print(nSerialNo, HEX);
        Serial.println();
    }
    else
    {
        Serial.println("Failed to read serial number.");
    }

    delay(1000);
}

retval readMeasurement(int16_t* pnVal/*TODO: , bool bSendCmd*/)
{
    if( SENSOR_SUCCESS != sendStartMeasurementCmd() )
    {
        return SENSOR_FAIL;
    }
    else if( SENSOR_SUCCESS != readMeasurementValue(pnVal) )
    {
        return SENSOR_FAIL;
    }

    return SENSOR_SUCCESS;
}

retval sendStartMeasurementCmd(void)
{
    Wire.beginTransmission(g_nDeviceAddress); // transmit to device
    Wire.write(0x10);
    Wire.write(0x00);
    Wire.endTransmission(); // stop transmitting

    return SENSOR_SUCCESS;
}

retval readMeasurementValue(int16_t* pnVal)
{
    uint8_t nReceived = 0;
    uint16_t nVal = 0;

    Wire.requestFrom(g_nDeviceAddress, (uint8_t)3); // request 3 bytes from slave device
  
    while( Wire.available() ) // slave may send less than requested
    {
        ++nReceived;

        char c = Wire.read(); // receive a byte as character
        // TODO: buffer raw bytes here! then check the CRC; convert to int16_t and return it
    }

    if( 3 != nReceived )
    {
        *pnVal = 0xDEAD;
        return SENSOR_FAIL;
    }

    *pnVal = nVal;
    return SENSOR_SUCCESS;
}

retval readSerialNumber(int32_t* pnSerialNo)
{
    if( SENSOR_SUCCESS != sendReadSerialNumberCmd() )
    {
        return SENSOR_FAIL; 
    }
    if( SENSOR_SUCCESS != readSerialNumberValue(pnSerialNo) )
    {
        return SENSOR_FAIL;
    }

    return SENSOR_SUCCESS;
}

retval sendReadSerialNumberCmd(void)
{
    Wire.beginTransmission(g_nDeviceAddress); // transmit to device
    Wire.write(0x31);
    Wire.write(0xAE);
    Wire.endTransmission(); // stop transmitting

    return SENSOR_SUCCESS;
}

retval readSerialNumberValue(int32_t* pnSerialNo)
{
    uint8_t nReceived = 0;
    uint32_t nSerialNo = 0;

    Wire.requestFrom(g_nDeviceAddress, (uint8_t)6); // request 6 bytes from slave device
  
    while( Wire.available() ) // slave may send less than requested
    {
        ++nReceived;

        char c = Wire.read(); // receive a byte as character
        // TODO: do something with the serial number here!
    }

    if( 6 != nReceived )
    {
        *pnSerialNo = 0xDEADBEEF;
        return SENSOR_FAIL;
    }

    *pnSerialNo = nSerialNo;
    return SENSOR_SUCCESS;
}

