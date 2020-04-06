#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_BME680.h>
#include <Wire.h>

String matrixausgabe_text  = " "; // Ausgabetext als globale Variable

volatile int matrixausgabe_index = 0;// aktuelle Position in Matrix

float dpPEEP = 0.0 ;
float dpMax = 0.0 ;
int HubMax = 0 ;
int Inspirationszeit = 0 ;
int Expirationszeit = 0 ;
bool motoron= false ;
int speed = 0 ;
// Control Feather Motor Shield
void setArduMotorSteps(int gpio_puls, int gpio_dir, int steps, int micro) {
  int dir=0; 	
  pinMode(gpio_puls,OUTPUT);
  pinMode(gpio_dir,OUTPUT);
  if (steps > 0)
    dir = 1;
  else {
    dir = 0;
    steps = - steps;
  }
  digitalWrite(gpio_dir,dir);
  for(int Index = 0; Index < steps; Index++) {
    digitalWrite(gpio_puls,HIGH);
    delayMicroseconds(micro);
    digitalWrite(gpio_puls,LOW);
    delayMicroseconds(micro);
  }
}

int startposition = 0 ;
float pEnviro = 0.0 ;
// BME680 Lib written by Limor Fried & Kevin Townsend for Adafruit Industries, http://www.adafruit.com/products/3660
Adafruit_BME680 boschBME680; // Objekt Bosch Umweltsensor
int boschBME680_ready = 0;

int Startzeit = 0 ;
int HubSchritte = 0 ;
float p = 0.0 ;
float pmax = 0.0 ;
float pmin = 0.0 ;


void setup(){ // Einmalige Initialisierung
  Serial.begin(115200);
  Wire.begin(); // ---- Initialisiere den I2C-Bus 

  if (Wire.status() != I2C_OK) Serial.println("Something wrong with I2C");

  boschBME680_ready = boschBME680.begin(118);

  if (boschBME680_ready == 0) {
    while(1) { 
      Serial.println("BME680 nicht vorhanden - der alte Octopus nutzt BME280, ggf. Puzzleteile tauschen");
      delay(500);
    }
  }

  // Set up Bosch BME 680
  boschBME680.setTemperatureOversampling(BME680_OS_8X);
  boschBME680.setHumidityOversampling(BME680_OS_2X);
  boschBME680.setPressureOversampling(BME680_OS_4X);
  boschBME680.setIIRFilterSize(BME680_FILTER_SIZE_3);
  boschBME680.setGasHeater(320, 150); // 320*C for 150 ms

  Serial.println();
  //------------ WLAN initialisieren 
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  delay(100);
  Serial.print ("\nWLAN connect to:");
  Serial.print("YOURSSID");
  WiFi.begin("YOURSSID","YOURPWD");
  while (WiFi.status() != WL_CONNECTED) { // Warte bis Verbindung steht 
    delay(500); 
    Serial.print(".");
  };
  Serial.println ("\nconnected, meine IP:"+ WiFi.localIP().toString());
  matrixausgabe_text = " Meine IP:" + WiFi.localIP().toString();
  matrixausgabe_index=0;

  Blynk.config("YOURKEY","blynk-cloud.com",80);// Downloads, docs, tutorials: http://www.blynk.cc
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

  dpPEEP = 1 ;

  dpMax = 70 ;

  HubMax = 12 ;

  Inspirationszeit = 10000 ;

  Expirationszeit = 2000 ;

  motoron = HIGH ;

  speed = 1000 ;

}

void loop() { // Kontinuierliche Wiederholung 
  Blynk.run();// Blynk Housekeeping
  calibrate();
  Inspiration();
  Expiration();
}

void calibrate()
{
  while ( ( ( analogRead(0) ) == ( 1024 ) ) )
  {
    yield(); // Aufruf Scheduler, bedient WLAN-Stack
    setArduMotorSteps(12,14,10,150);    // Ausgabe an Motor 
  }

  for (startposition= 1; startposition<= ( 200 ); startposition=startposition+1)
  {
    setArduMotorSteps(12,14,-10,150);    // Ausgabe an Motor 
  }
  pEnviro = (boschBME680.readPressure()/100.+(0.0)) ;
  delay( 100 );
}

void Inspiration()
{
  Startzeit = millis() ;
  HubSchritte = 0 ;
  p = ( (boschBME680.readPressure()/100.+(0.0)) - pEnviro ) ;
  pmax = 0 ;
  Serial.print("Inspiration -start");
  Serial.println();
  while ( ( ( ( ( p ) < ( dpMax ) ) && ( ( HubSchritte ) < ( HubMax ) ) ) && ( ( ( millis() - Startzeit ) ) < ( Inspirationszeit ) ) ) )
  {
    yield(); // Aufruf Scheduler, bedient WLAN-Stack
    if (motoron)
    {
      setArduMotorSteps(12,14,-1000,150);    // Ausgabe an Motor 
      delay( 1 );
    }
    HubSchritte = ( HubSchritte + 1 ) ;
    p = ( (boschBME680.readPressure()/100.+(0.0)) - pEnviro ) ;
    if (( ( p ) > ( pmax ) ))
    {
      pmax = p ;
    }
    Serial.print("T:"+String(String(( millis() - Startzeit ))));
    Serial.print(",H:"+String(String(HubSchritte)));
    Serial.print(",P:"+String(String(p)));
    Serial.println();
    Blynk.virtualWrite(1,p);// Wert an Blynk-Server übermitteln
    Blynk.run();// Blynk Housekeeping
    Blynk.virtualWrite(2,( HubSchritte * 37.5 ));// Wert an Blynk-Server übermitteln
    Blynk.run();// Blynk Housekeeping
    delay( p );
  }

  while ( ( ( ( millis() - Startzeit ) ) < ( Inspirationszeit ) ) )
  {
    yield(); // Aufruf Scheduler, bedient WLAN-Stack
    Serial.print(".");
    delay( 1 );
  }

  Serial.print(".");
  Serial.println();
  Serial.print("Inspriration -complete: maxP:"+String(String(pmax)));
  Serial.print("maxHub:"+String(String(HubSchritte)));
  Serial.println();
  delay( 100 );
}

void Expiration()
{
  Startzeit = millis() ;
  p = ( (boschBME680.readPressure()/100.+(0.0)) - pEnviro ) ;
  pmin = 0 ;
  Serial.print("Expiration -start");
  Serial.println();
  Serial.print("P:"+String(String(p)));
  Serial.print(",BEEP:"+String(String(dpPEEP)));
  Serial.println();
  delay( p );
  while ( ( ( ( p ) >= ( dpPEEP ) ) && ( ( HubSchritte ) > ( 0 ) ) ) )
  {
    yield(); // Aufruf Scheduler, bedient WLAN-Stack
    if (motoron)
    {
      setArduMotorSteps(12,14,1000,150);    // Ausgabe an Motor 
      delay( 1 );
      HubSchritte = ( HubSchritte - 1 ) ;
      p = ( (boschBME680.readPressure()/100.+(0.0)) - pEnviro ) ;
      if (( ( p ) < ( pmin ) ))
      {
        pmin = p ;
      }
      Serial.print("T:"+String(String(( millis() - Startzeit ))));
      Serial.print(",H:"+String(String(HubSchritte)));
      Serial.print(",P:"+String(String(pmin)));
      Serial.println();
    }
  }

  while ( ( ( ( millis() - Startzeit ) ) < ( Expirationszeit ) ) )
  {
    yield(); // Aufruf Scheduler, bedient WLAN-Stack
    Serial.print(".");
    delay( 1 );
  }

  Serial.print(".");
  Serial.println();
  Serial.print("Expiration -completed");
  Serial.println();
}

