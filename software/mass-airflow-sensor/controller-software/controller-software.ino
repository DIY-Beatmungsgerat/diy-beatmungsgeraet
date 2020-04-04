#include <Wire.h>
#include "crc8.h"

// Test setup:
//   Arduino Leonardo:
//     I2C SDA:      2
//     I2C SCL:      3
//     Real-time supervision pin: 4

/* define the address of the sensor on the I2C bus */
uint8_t g_nDeviceAddress = 64;

/* Mapping of the digital output pin for real-time supervision, define alias here */
#define RT_SUPERVISION_PIN    (4)

/* Define 'DEBUG' in order to get additional output via the serial console;
 * undefine for speed-up and potential higher rates
 */
#define DEBUG
#ifdef DEBUG
    #define debugPrint    Serial.print
    #define debugPrintln  Serial.println
#else
    #define debugPrint    
    #define debugPrintln  
#endif

/* Definition of interval (in milliseconds) to query the sensor via I2C for new measurement.
 * Current setting: query approx. every 20 ms, i.e. at a rate of 50 Hz.
 * Note: This is not exact as there will be plenty of overhead.
  *      When DEBUG output is active, it takes about 1 ms per cycle.
  *      When it's not active, it will take approx. 0.8 ms per cycle also, so subtract 1 ms here.
 * Caution: The interrupt overhead is not subtracted automatically.
 */
#define SENSOR_QUERY_INTERVAL (10u)
#ifdef DEBUG
    #define SENSOR_LOOP_DELAY (SENSOR_QUERY_INTERVAL - (1u))
#else
    #define SENSOR_LOOP_DELAY (SENSOR_QUERY_INTERVAL - (1u))
#endif

/* typedef return values of the sensor API */
enum eRetVal { SENSOR_SUCCESS = 0, SENSOR_FAIL, SENSOR_CRC_ERROR, SENSOR_CMD_ERROR, SENSOR_RXCNT_ERROR, SENSOR_PARAM_ERROR };

eRetVal readMeasurement(float* pfFlow, int16_t* pnRaw, bool bSendMeasCmd);
eRetVal readSerialNumber(int32_t* pnSerialNo);

void setup()
{
    uint32_t nSensorSerialNo = 0xBADC0FFE;

    delay(3000); /* delay added for debugging so that the start of the serial transmission is not missed */

    Serial.begin(230400); // start serial for debug output

    Wire.begin();

#ifdef RT_SUPERVISION_PIN
    pinMode(RT_SUPERVISION_PIN, OUTPUT);
#endif

    debugPrintln("Finished setup.");

    if( SENSOR_SUCCESS == readSerialNumber(&nSensorSerialNo) )
    {
        debugPrint("Read serial number: ");
        debugPrint(nSensorSerialNo, HEX);
        debugPrintln("");
    }
    else
    {
        debugPrintln("Failed to read serial number.");
    }
}

void loop()
{
    static bool bSendMeasCommand = true;
    static float fFlow = 0.0f;
    static eRetVal eStatus = SENSOR_FAIL;
#ifdef DEBUG
    static uint16_t nRaw = 0;
#endif

#ifdef RT_SUPERVISION_PIN
    digitalWrite(RT_SUPERVISION_PIN, HIGH);
#endif

#ifdef DEBUG
    eStatus = readMeasurement(&fFlow, &nRaw, bSendMeasCommand);
#else
    eStatus = readMeasurement(&fFlow, NULL, bSendMeasCommand);
#endif

    switch( eStatus )
    {
        case SENSOR_SUCCESS:
            /*
            debugPrint("Raw: ");
            debugPrint( nRaw );
            debugPrint(", ");
            */
            debugPrint("Flow: ");
            debugPrint( fFlow );
            debugPrintln(" [slm]");
    
            bSendMeasCommand = false; /* do not resend measurement command after first success */
            break;
        case SENSOR_RXCNT_ERROR:
            debugPrintln("[ERROR] Receive count error.");
            break;
        case SENSOR_CRC_ERROR:
            debugPrintln("[ERROR] CRC error.");
            break;
        default:
            debugPrintln("[ERROR] Measurement error.");
            break;
    }

#ifdef RT_SUPERVISION_PIN
    digitalWrite(RT_SUPERVISION_PIN, LOW);
#endif

    delay(SENSOR_LOOP_DELAY);
}

crc_t calcCrc(const unsigned char *pData, size_t nDataLen)
{
    crc_t nCrc = crc_init();
    nCrc = crc_update(nCrc, pData, nDataLen);
    nCrc = crc_finalize(nCrc);

    return nCrc;
}

eRetVal readMeasurement(float* pfFlow, uint16_t* pnRaw, bool bSendMeasCmd)
{
    static eRetVal eStatus;
    static uint16_t nVal;

    if( bSendMeasCmd )
    {
        if( SENSOR_SUCCESS != sendStartMeasurementCmd() )
        {
            return SENSOR_CMD_ERROR;
        }
    }

    eStatus = readMeasurementValue(&nVal);
    if( eStatus == SENSOR_SUCCESS )
    {
        if( NULL != pnRaw )
        {
            *pnRaw = nVal;
        }
        convertToFlow(&nVal, pfFlow);
    }

    return eStatus;
}

eRetVal sendStartMeasurementCmd(void)
{
    Wire.beginTransmission(g_nDeviceAddress); // transmit to device
    Wire.write(0x10);
    Wire.write(0x00);
    Wire.endTransmission(); // stop transmitting

    return SENSOR_SUCCESS;
}

eRetVal convertToFlow(uint16_t* pnVal, float* pfFlow)
{
    static const uint16_t fOffsetFlow = 32000.0f;
    static const float fScaleFactor = 140.0f;

    if( NULL == pnVal || NULL == pfFlow )
    {
        return SENSOR_PARAM_ERROR;
    }
    *pfFlow = ((float)(*pnVal) - fOffsetFlow) / fScaleFactor;

    return SENSOR_SUCCESS;
}

eRetVal readMeasurementValue(int16_t* pnVal)
{
    static char sHexBuf[12]; /* string buffer for debug output via UART */
    static uint8_t nMeasRx[3];
    uint8_t nReceived = 0;
    int16_t nVal = 0;

    Wire.requestFrom(g_nDeviceAddress, (uint8_t)3); // request 3 bytes from slave device
  
    while( Wire.available() ) // slave may send less than requested
    {
        uint8_t cRxByte = Wire.read(); // receive a byte as character
        if( nReceived < 3 )
        {
            // buffer raw bytes and received CRC here
            nMeasRx[nReceived] = cRxByte;
        }
        ++nReceived;
    }

    if( 3 != nReceived )
    {
        *pnVal = 0xDEAD;
        debugPrint("[ERROR] Received ");
        debugPrint( nReceived );
        debugPrintln(" instead of 3 bytes.");
        return SENSOR_RXCNT_ERROR;
    }

    // calculate CRC here and check with received CRC
    crc_t cCalculatedCrc = calcCrc(&nMeasRx[0], 2);

    if( nMeasRx[2] != (uint8_t)cCalculatedCrc )
    {
        debugPrintln("[ERROR] CRC does not match.");
        return SENSOR_CRC_ERROR;
    }

    // convert to int16_t and prepare as return value
    nVal = ((uint16_t)nMeasRx[0] << 8) | (uint16_t)nMeasRx[1];

    *pnVal = nVal;
    return SENSOR_SUCCESS;
}

eRetVal readSerialNumber(int32_t* pnSerialNo)
{
    if( SENSOR_SUCCESS != sendReadSerialNumberCmd() )
    {
        return SENSOR_CMD_ERROR; 
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

    debugPrint("Raw serial number data: ");
    while( Wire.available() ) // slave may send less than requested
    {
        ++nReceived;

        uint8_t cRxByte = Wire.read(); // receive a byte as character
        // TODO: do something with the serial number here!
        //       i.e. prepare 32 bit return value and check CRC

        /* print every byte as two hex digits with prefix '0x', NULL-terminate the string */
        sprintf(sHexBuf, "0x%02X ", cRxByte);
        sHexBuf[5] = 0;
        debugPrint(sHexBuf);
    }
    debugPrintln("");

    if( 6 != nReceived )
    {
        *pnSerialNo = 0xDEADBEEF;
        return SENSOR_FAIL;
    }

    *pnSerialNo = nSerialNo;
    return SENSOR_SUCCESS;
}

