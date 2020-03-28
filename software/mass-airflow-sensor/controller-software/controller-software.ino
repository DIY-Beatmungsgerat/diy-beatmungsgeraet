#include <Wire.h>
#include "crc8.h"

// Test setup:
//   Arduino Leonardo:
//     pin asssignment 2 (SDA), 3 (SCL)

/* define the address of the sensor on the I2C bus */
uint8_t g_nDeviceAddress = 64;

/* Definition of interval (in milliseconds) to query the sensor via I2C for new measurement.
 * Current setting: query approx. every 50 ms, i.e. at a rate of 20 Hz.
 * Note: This is not exact as there will be plenty of overhead.
 */
#define SENSOR_QUERY_INTERVAL (50u)

/* typedef return values of the sensor API */
enum eRetVal { SENSOR_SUCCESS = 0, SENSOR_FAIL, SENSOR_CRC_ERROR };

eRetVal readMeasurement(int16_t* pnVal/*TODO: , bool bSendCmd*/);
eRetVal readSerialNumber(int32_t* pnSerialNo);

void setup()
{
    uint32_t nSensorSerialNo = 0xBADC0FFE;

    delay(3000); /* delay added for debugging so that the start of the serial transmission is not missed */

    Wire.begin();
    Serial.begin(230400); // start serial for debug output

    Serial.println("Finished setup.");

    if( SENSOR_SUCCESS == readSerialNumber(&nSensorSerialNo) )
    {
        Serial.print("Read serial number: ");
        Serial.print(nSensorSerialNo, HEX);
        Serial.println();
    }
    else
    {
        Serial.println("Failed to read serial number.");
    }
}

void loop()
{
    uint16_t nVal = 0;

    if( SENSOR_SUCCESS == readMeasurement(&nVal) )
    {
        //Serial.print("Read measurement; raw (decimal): ");
        //Serial.println( nVal );
        Serial.print("Measurement: ");
        Serial.print( nVal >> 2 );
        Serial.println(" mmH2O");
        /* TODO/FIXME: measurement will change to volume flow, currently it's differential pressure */
    }
    else
    {
        Serial.println("Failed to read measurement");
    }

    delay(SENSOR_QUERY_INTERVAL);
}

crc_t calcCrc(const unsigned char *pData, size_t nDataLen)
{
    crc_t nCrc = crc_init();
    nCrc = crc_update(nCrc, pData, nDataLen);
    nCrc = crc_finalize(nCrc);

    return nCrc;
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
    int16_t nVal = 0;
    char sHexBuf[12]; /* string buffer for debug output via UART */
    uint8_t nMeasRx[3];

    Wire.requestFrom(g_nDeviceAddress, (uint8_t)3); // request 3 bytes from slave device
  
//    Serial.print("Raw measurement data: ");
    while( Wire.available() ) // slave may send less than requested
    {
        uint8_t cRxByte = Wire.read(); // receive a byte as character
        if( nReceived < 3 )
        {
            // buffer raw bytes and received CRC here
            nMeasRx[nReceived] = cRxByte;
        }
        ++nReceived;

//        /* print every byte as two hex digits with prefix '0x', NULL-terminate the string */
//        sprintf(sHexBuf, "0x%02X ", cRxByte);
//        sHexBuf[5] = 0;
//        Serial.print(sHexBuf);
    }
//    Serial.println();

    if( 3 != nReceived )
    {
        Serial.println("[ERROR] Did not receive 3 bytes.");
        *pnVal = 0xDEAD;
        return SENSOR_FAIL;
    }

    // calculate CRC here and check with received CRC
    crc_t cCalculatedCrc = calcCrc(&nMeasRx[0], 2);

//    Serial.print("Calculated CRC: ");
//    sprintf(sHexBuf, "0x%02X ", (uint8_t)cCalculatedCrc);
//    sHexBuf[4] = 0;
//    Serial.print(sHexBuf);
//
//    Serial.print(", received CRC: ");
//    sprintf(sHexBuf, "0x%02X ", (uint8_t)nMeasRx[2]);
//    sHexBuf[4] = 0;
//    Serial.print(sHexBuf);
//    Serial.println();

    if( nMeasRx[2] != (uint8_t)cCalculatedCrc )
    {
        Serial.println("[ERROR] CRC does not match.");
        return SENSOR_CRC_ERROR;
    }
//    else
//    {
//        Serial.println("[DEBUG] CRC check OK.");
//    }

    // convert to int16_t and prepare as return value
    nVal = ((int16_t)nMeasRx[0] << 8) | (int16_t)nMeasRx[1];
    
//    Serial.print("16 bit sensor reading: ");
//    Serial.println(nVal);

//    sprintf(sHexBuf, "0x%02X 0x%02X ", nMeasRx[0], nMeasRx[1]);
//    sHexBuf[11] = 0;
//    Serial.println(sHexBuf);

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
    char sHexBuf[6]; /* string buffer for debug output via UART */

    Wire.requestFrom(g_nDeviceAddress, (uint8_t)6); // request 6 bytes from slave device

    Serial.print("Raw serial number data: ");
    while( Wire.available() ) // slave may send less than requested
    {
        ++nReceived;

        uint8_t cRxByte = Wire.read(); // receive a byte as character
        // TODO: do something with the serial number here!
        //       i.e. prepare 32 bit return value and check CRC

        /* print every byte as two hex digits with prefix '0x', NULL-terminate the string */
        sprintf(sHexBuf, "0x%02X ", cRxByte);
        sHexBuf[5] = 0;
        Serial.print(sHexBuf);
    }
    Serial.println();

    if( 6 != nReceived )
    {
        *pnSerialNo = 0xDEADBEEF;
        return SENSOR_FAIL;
    }

    *pnSerialNo = nSerialNo;
    return SENSOR_SUCCESS;
}

