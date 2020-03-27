#include <Wire.h>

// Test setup:
//   Arduino Leonardo:
//     pin asssignment 2 (SDA), 3 (SCL)

/* define the address of the sensor on the I2C bus */
uint8_t g_nDeviceAddress = 64;

/* typedef return values of the sensor API */
enum eRetVal { SENSOR_SUCCESS = 0, SENSOR_FAIL };

void setup()
{
    Wire.begin();
    Serial.begin(230400); // start serial for debug output

    Serial.println("Finished setup.");
}

void loop()
{
    uint16_t nVal = 0;
    uint32_t nSerialNo = 0xBADC0FFE;

    Serial.println("Loop");
//    if( SENSOR_SUCCESS == readMeasurement(&nVal) )
//    {
//        Serial.print("Read measurement; raw: ");
//        Serial.print(nVal, HEX);
//        Serial.println();
//    }
//    else
//    {
//        Serial.println("Failed to read measurement");
//    }

    if( SENSOR_SUCCESS == txOnly1() )
    {
        Serial.println("Tx1 OK");
    }
    else
    {
        Serial.println("Tx1 NOK");
    }
    delay(2000);

//    if( SENSOR_SUCCESS == readSerialNumber(&nSerialNo) )
//    {
//        Serial.print("Read serial number: ");
//        Serial.print(nSerialNo, HEX);
//        Serial.println();
//    }
//    else
//    {
//        Serial.println("Failed to read serial number.");
//    }

    if( SENSOR_SUCCESS == rxOnly1() )
    {
        Serial.println("Rx1 OK");
    }
    else
    {
        Serial.println("Rx1 NOK");
    }
    delay(4000);

    if( SENSOR_SUCCESS == txOnly2() )
    {
        Serial.println("Tx2 OK");
    }
    else
    {
        Serial.println("Tx2 NOK");
    }
    delay(2000);

    if( SENSOR_SUCCESS == rxOnly2() )
    {
        Serial.println("Rx2 OK");
    }
    else
    {
        Serial.println("Rx2 NOK");
    }
    delay(4000);
}

eRetVal txOnly1(void)
{
    Serial.println("Tx1");
    Wire.beginTransmission(g_nDeviceAddress); // transmit to device
    Wire.write(0x10);
    Wire.write(0x00);
    Wire.endTransmission(); // stop transmitting

    return SENSOR_SUCCESS;
}

eRetVal txOnly2(void)
{
    Serial.println("Tx2");
    Wire.beginTransmission(g_nDeviceAddress); // transmit to device
    Wire.write(0x31);
    Wire.write(0xAE);
    Wire.endTransmission(); // stop transmitting

    return SENSOR_SUCCESS;
}

eRetVal rxOnly1(void)
{
    uint8_t nReceived = 0;
    uint16_t nVal = 0;

    Serial.println("Rx1");
    Wire.requestFrom(g_nDeviceAddress, (uint8_t)3); // request 3 bytes from slave device
  
    while( Wire.available() ) // slave may send less than requested
    {
        ++nReceived;

        char c = Wire.read(); // receive a byte as character
        // TODO: buffer raw bytes here! then check the CRC; convert to int16_t and return it
    }

    if( 3 != nReceived )
    {
        return SENSOR_FAIL;
    }

    return SENSOR_SUCCESS;
}

eRetVal rxOnly2(void)
{
    uint8_t nReceived = 0;
    uint16_t nVal = 0;

    Serial.println("Rx2");
    Wire.requestFrom(g_nDeviceAddress, (uint8_t)6); // request 6 bytes from slave device
  
    while( Wire.available() ) // slave may send less than requested
    {
        ++nReceived;

        char c = Wire.read(); // receive a byte as character
        // TODO: buffer raw bytes here! then check the CRC; convert to int16_t and return it
    }

    if( 6 != nReceived )
    {
        return SENSOR_FAIL;
    }

    return SENSOR_SUCCESS;
}

eRetVal readMeasurement(int16_t* pnVal/*TODO: , bool bSendCmd*/)
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

eRetVal sendStartMeasurementCmd(void)
{
    Wire.beginTransmission(g_nDeviceAddress); // transmit to device
    Wire.write(0x10);
    Wire.write(0x00);
    Wire.endTransmission(); // stop transmitting

    return SENSOR_SUCCESS;
}

eRetVal readMeasurementValue(int16_t* pnVal)
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

eRetVal readSerialNumber(int32_t* pnSerialNo)
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

eRetVal sendReadSerialNumberCmd(void)
{
    Wire.beginTransmission(g_nDeviceAddress); // transmit to device
    Wire.write(0x31);
    Wire.write(0xAE);
    Wire.endTransmission(); // stop transmitting

    return SENSOR_SUCCESS;
}

eRetVal readSerialNumberValue(int32_t* pnSerialNo)
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

