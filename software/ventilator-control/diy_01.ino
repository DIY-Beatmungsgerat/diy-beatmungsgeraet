#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Encoder.h>
#include <bsec.h>
#include <Ticker.h>

String matrixausgabe_text  = " "; // Ausgabetext als globale Variable

volatile int matrixausgabe_index = 0;// aktuelle Position in Matrix

int breathcycle = 0 ;
int breathpause = 0 ;
int speed = 0 ;
bool start= false ;
int humdity = 0 ;
int volumen = 0 ;
// Feather Adafruit Motor Shield v2 http:\www.adafruit.com/products/1438
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Treiber-Objekt
Adafruit_StepperMotor *myStepMotor1 = AFMS.getStepper(100,1); // Motor 1-Objekt, 100 Steps/Umdrehung
Adafruit_StepperMotor *myStepMotor2 = AFMS.getStepper(100,2); // Motor 2-Objekt
// Control Feather Motor Shield
void setMotorSteps(int mot, int steps, int speed) {
  int dir= RELEASE; 	
  if (steps > 0)
    dir = FORWARD;
  else {
    dir = BACKWARD;
    steps = - steps;
  }
  switch (mot) {
  case 1:
    myStepMotor1->setSpeed(speed);
    myStepMotor1->step(steps, dir, DOUBLE);
    if (steps==0) myStepMotor1->release();
    break;
  case 2:
    myStepMotor2->setSpeed(speed);
    myStepMotor2->step(steps, dir, DOUBLE);
    if (steps==0) myStepMotor2->release();
    break;
  }
}

// Encoder-Library http://www.pjrc.com/teensy/td_libs_Encoder.html
Encoder button_encoder(14,12); // Objekt Dreh-Encoder
/* 
 Bosch BSEC Lib, https://github.com/BoschSensortec/BSEC-Arduino-library
 The BSEC software is only available for download or use after accepting the software license agreement.
 By using this library, you have agreed to the terms of the license agreement: 
 https://ae-bst.resource.bosch.com/media/_tech/media/bsec/2017-07-17_ClickThrough_License_Terms_Environmentalib_SW_CLEAN.pdf */
Bsec iaqSensor;     // Create an object of the class Bsec 
Ticker Bsec_Ticker; // schedule cyclic update via Ticker 

// ------------------------   Helper functions Bosch Bsec - Lib 
void checkIaqSensorStatus(void)
{ 
  String output; 
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      for (;;) {
        Serial.println(output);
        delay(500);
      } // Halt in case of failure 
    } 
    else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      for (;;){
        Serial.println(output);
        delay(500);
      }  // Halt in case of failure 
    } 
    else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}

// Housekeeping: scheduled update using ticker-lib
void iaqSensor_Housekeeping(){  // get new data 
  iaqSensor.run();
}

int pressure = 0 ;
int co2 = 0 ;
int heardbeat = 0 ;
int spo2 = 0 ;
int heartbeat = 0 ;


void setup(){ // Einmalige Initialisierung
  Serial.begin(115200);
  Serial.println();
  Wire.begin(); // ---- Initialisiere den I2C-Bus 

  if (Wire.status() != I2C_OK) Serial.println("Something wrong with I2C");

  AFMS.begin(); // Setup Feather-Shield 1.6KHz PWM

  Wire.setClock(400000L); // speed up i2c 
  iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
  String output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();

  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();  
  iaqSensor_Housekeeping();
  Bsec_Ticker.attach_ms(2000, iaqSensor_Housekeeping);

  randomSeed(analogRead(A0) + analogRead(A0) + analogRead(A0));
  pinMode( 15 , OUTPUT);
  //------------ WLAN initialisieren 
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  delay(100);
  Serial.print ("\nWLAN connect to:");
  Serial.print("your ssid");
  WiFi.begin("your ssid","your pwd");
  while (WiFi.status() != WL_CONNECTED) { // Warte bis Verbindung steht 
    delay(500); 
    Serial.print(".");
  };
  Serial.println ("\nconnected, meine IP:"+ WiFi.localIP().toString());
  matrixausgabe_text = " Meine IP:" + WiFi.localIP().toString();
  matrixausgabe_index=0;

  Blynk.config("key","blynk-cloud.com",80);// Downloads, docs, tutorials: http://www.blynk.cc
  int BlynkCon = 0;
  while (BlynkCon == 0) {
    Serial.print ("\nBlynk connect ... ");
    BlynkCon=Blynk.connect();
    if (BlynkCon == 0) {
      Serial.println("failed, try again");
      delay(1000);
    }
  }
  Serial.println("connected");

  breathcycle = 7000 ;

  breathpause = 3500 ;

  speed = 50 ;

  start = LOW ;

}

void loop() { // Kontinuierliche Wiederholung 
  Blynk.run();// Blynk Housekeeping
  readsensor();
  senddata();
  if (( ( ( humdity ) > ( 60 ) ) && start ))
  {
    Serial.print("Beatmung start - Volumen:"+String(String(volumen)));
    Serial.println();
    delay( 250 );
    setMotorSteps(1,volumen,speed);    // Ausgabe an Motor 
    delay( breathpause );
    setMotorSteps(1,( volumen * -1 ),speed);    // Ausgabe an Motor 
    delay( 250 );
  }
}

void readsensor()
{
  volumen = button_encoder.read() ;
  humdity = iaqSensor.humidity ;
  pressure = (iaqSensor.pressure/100.) ;
  co2 = iaqSensor.co2Equivalent ;
  heardbeat = ( 70 + 	random( 20 ) ) ;
  spo2 = ( 99 - 	random( 20 ) ) ;
  if (digitalRead(2)==LOW)
  {
    start = HIGH ;
  }
  Serial.print("Volumen:"+String(String(volumen)));
  Serial.println();
  Serial.print("Start:"+String(String(start)));
  Serial.println();
  checkalarm();
}

void senddata()
{
  Blynk.virtualWrite(1,volumen);// Wert an Blynk-Server übermitteln
  Blynk.run();// Blynk Housekeeping
  Blynk.virtualWrite(2,humdity);// Wert an Blynk-Server übermitteln
  Blynk.run();// Blynk Housekeeping
  Blynk.virtualWrite(3,pressure);// Wert an Blynk-Server übermitteln
  Blynk.run();// Blynk Housekeeping
  Blynk.virtualWrite(4,co2);// Wert an Blynk-Server übermitteln
  Blynk.run();// Blynk Housekeeping
  Blynk.virtualWrite(5,heardbeat);// Wert an Blynk-Server übermitteln
  Blynk.run();// Blynk Housekeeping
  Blynk.virtualWrite(6,spo2);// Wert an Blynk-Server übermitteln
  Blynk.run();// Blynk Housekeeping
  Blynk.virtualWrite(7,start);// Wert an Blynk-Server übermitteln
  Blynk.run();// Blynk Housekeeping
}

void checkalarm()
{
  if (( ( ( co2 ) < ( 400 ) ) || ( ( spo2 ) < ( 80 ) ) ))
  {
    Serial.print("Alarm SP02:"+String(String(spo2)));
    Serial.print(",CO2:"+String(String(co2)));
    Serial.print(",heartbeat:"+String(String(heartbeat)));
    Serial.println();
    digitalWrite( 15 , HIGH );
    delay( 5000 );
    digitalWrite( 15 , LOW );
  }
}
