#include <Wire.h>

// Test setup:
//   Arduino Nano:
//     pin asssignment A4 (SDA), A5 (SCL)

void setup()
{
    // TODO: decide address to be used via GPIO input pin
    Wire.begin(4);                // join i2c bus with address #4
    Wire.onReceive(receiveEvent); // register event handler
    Serial.begin(9600);           // start serial for debug output
}

void loop()
{
    delay(100);
    // TODO: read analog input to get measurement from sensor
}

void receiveEvent(int nBytes)
{
    Serial.print("Rx: ");
    while(0 < Wire.available())   // loop through all but the last
    {
        char cRxByte = Wire.read(); // receive byte as a character
        Serial.print(cRxByte);    // print the character
    }
    Serial.println("");           // print the integer
}
