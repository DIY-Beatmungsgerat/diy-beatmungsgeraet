#include <Wire.h>

// Test setup:
//   Arduino Leonardo:
//     pin asssignment 2 (SDA), 3 (SCL)

void setup()
{
    Wire.begin();
}

void loop()
{
    Wire.beginTransmission(4); // transmit to device #4
    Wire.write("test");        // sends four bytes
    Wire.endTransmission();    // stop transmitting
    Serial.println("Tx: test");
    
    delay(1000);
}

