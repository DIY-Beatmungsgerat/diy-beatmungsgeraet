#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <WebSocketsServer.h>           //Bibliothek von Markus Sattler installieren 
#include <ArduinoOTA.h>
#include <stdlib.h>
#include <stdio.h> 
#include <string.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//WLAN DATEN
#ifndef STASSID
#define STASSID "WLAN SSID"                      //Hier die WLAN Credentials eintragen 
#define STAPSK  "WLAN PASSWORT"                      
#endif
const char* ssid     = STASSID;
const char* password = STAPSK;

//MOTOR VARIABLEN
int PIN_STEPPER_CLK = 19;  //CLOCK
int PIN_STEPPER_CW = 18;   //DIRECTION
int clk_pulses_to_go;
int last_motor_clk_pulse;
int clk_pulses_to_go_anlauf;
int clk_pulses_to_go_after_anlauf; 
int clk_pulses_delay_anlauf;
int clk_pulses_delay_after_anlauf;  
bool anlauf_fertig;
float time_measurement_started;
int cur_motor_position;
int cur_motor_direction;

int steps_to_go_cur_run; //wieviele Schritte sind im Lauf?
int step_counter_cur_run; 

int motor_speed_phases_at_what_step_speed_changes[10];    //bei welcher Schrittzahl soll die Geschwindigkeit gewechselt weden   = (0,100,200,500,1000);
int motor_speed_phases_at_what_step_which_speed[10];      //was ist dann die neue Schrittzahl (600,400,350,250,200)
int cur_speed_step;


//SENSOR VARIABLEN
Adafruit_BME280 bme;

//SONSTIGE VARIABLEN
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
WebSocketsServer webSocket = WebSocketsServer(8081);
int measurements_should_be_send;

void IRAM_ATTR onTimer();

void setup() 
{
  Serial.begin(115200);

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  ArduinoOTA.setHostname("myesp32_motordriver");
  ArduinoOTA.setPassword("test3");
  ArduinoOTA.begin();

  pinMode(PIN_STEPPER_CLK, OUTPUT);
  pinMode(PIN_STEPPER_CW, OUTPUT);
  
  if (bme.begin(0x76) != true) 
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    delay(2500); 
  }
  
  clk_pulses_to_go=0;
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  time_measurement_started = 0;

  cur_motor_position = 0;


   bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X1,   // temperature
                    Adafruit_BME280::SAMPLING_X4,   // pressure
                    Adafruit_BME280::SAMPLING_NONE, // humidity
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5 );

  measurements_should_be_send = 0;
}

void loop()
{
  ArduinoOTA.handle();

  webSocket.loop();

    int cur_pressure_in_kolben = (int)( (bme.readPressure() / 100.0F)*1000 ); 
    int cur_time = (millis() - time_measurement_started);   

    char buffer[10];
    String msg_to_send;
    itoa(cur_time,buffer,10);    
    msg_to_send = String(buffer) + String(";"); 

    itoa(cur_motor_position,buffer,10);
    msg_to_send = msg_to_send + String(buffer) + String(";");

    itoa(cur_pressure_in_kolben,buffer,10);
    msg_to_send = msg_to_send + String(buffer) + String(";");

    if (measurements_should_be_send == 1)
    {
      webSocket.broadcastTXT(msg_to_send);
    }
     

  delay(10);
}

/*
motor_speed_phases_at_what_step_speed_changes[0]=0;
motor_speed_phases_at_what_step_speed_changes[1]=50;
motor_speed_phases_at_what_step_speed_changes[2]=100;
motor_speed_phases_at_what_step_speed_changes[3]=200;
motor_speed_phases_at_what_step_speed_changes[4]=300;
motor_speed_phases_at_what_step_speed_changes[5]=2000;
motor_speed_phases_at_what_step_speed_changes[6]=3000;
motor_speed_phases_at_what_step_speed_changes[7]=4000;
  
motor_speed_phases_at_what_step_which_speed[0] = 600;
motor_speed_phases_at_what_step_which_speed[1] = 400;
motor_speed_phases_at_what_step_which_speed[2] = 300;
motor_speed_phases_at_what_step_which_speed[3] = 250;
motor_speed_phases_at_what_step_which_speed[4] = 180;
motor_speed_phases_at_what_step_which_speed[5] = 150;
motor_speed_phases_at_what_step_which_speed[6] = 120;        
*/

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length)
{
   if (type == WStype_TEXT)
   {
      if (payload[0] == 'R') //RESET
      {
         time_measurement_started = millis(); 
         cur_motor_position = 0;
      }

      if (payload[0] == 'M') //Measurements sollen gesendet werden
      {
        measurements_should_be_send = 1; 
      }

      if (payload[0] == 'O') //Measurements sollen nicht gesendet werden
      {
        measurements_should_be_send = 0; 
      }


      if (payload[0] == 'D') //DRIVE
      {
        //Payload ins chr_buffer schreiben
        char chr_buffer[200];
        String str_buffer = (char*) payload;
        str_buffer.toCharArray(chr_buffer, 200);

        //Das JSON-Format aufbrechen
        int command_buffer[30];
        int counter = 0; 
        char *json = strtok (chr_buffer, ";");
        while(json != 0)
        {
           if (counter>=1)
           {  
              command_buffer[(counter-1)] = atoi(json);
           }
           json = strtok(0,";");
           counter = counter + 1;
        }

        int number_step_phases = (counter - 3) / 2;
        for (int i=1;i<=number_step_phases;i++)
        {
          motor_speed_phases_at_what_step_speed_changes[(i-1)] = command_buffer[(2*i)];
          motor_speed_phases_at_what_step_which_speed[(i-1)] = command_buffer[((2*i)+1)];
          Serial.print(motor_speed_phases_at_what_step_speed_changes[(i-1)]);
          Serial.print("---");
          Serial.println(motor_speed_phases_at_what_step_which_speed[(i-1)]);
        }  

        if (command_buffer[0] == 0) {digitalWrite(PIN_STEPPER_CW, LOW);cur_motor_direction = 1;}
        if (command_buffer[0] == 1) {digitalWrite(PIN_STEPPER_CW, HIGH); cur_motor_direction = -1;}
        steps_to_go_cur_run = command_buffer[1];
        step_counter_cur_run = 0;
        cur_speed_step = 0;
        
        timerAlarmWrite(timer,  motor_speed_phases_at_what_step_which_speed[0], true); //alle 600 us
        timerAlarmEnable(timer);
      }
   }
}

void IRAM_ATTR onTimer() 
{
  portENTER_CRITICAL_ISR(&timerMux);
  if (step_counter_cur_run <= steps_to_go_cur_run)
  {
    if (last_motor_clk_pulse==1) 
    {
        digitalWrite(PIN_STEPPER_CLK, LOW);   //einmal von LOW nach HIGH und dann wieder zu LOW ist eine Umdrehung
        last_motor_clk_pulse = 0;
        step_counter_cur_run = step_counter_cur_run + 1;
        cur_motor_position = cur_motor_position + cur_motor_direction;
    }else
    {
        digitalWrite(PIN_STEPPER_CLK, HIGH);
        last_motor_clk_pulse = 1;
        step_counter_cur_run = step_counter_cur_run + 1;      
        cur_motor_position = cur_motor_position + cur_motor_direction;
    }  
  }

  if (step_counter_cur_run == motor_speed_phases_at_what_step_speed_changes[(cur_speed_step+1)]) //Speedwechsel
  {
    if (step_counter_cur_run < steps_to_go_cur_run) //Wichtig: Hier darf kein Gleichheitzeichen stehen
    {
      timerAlarmWrite(timer, motor_speed_phases_at_what_step_which_speed[(cur_speed_step+1)], true); 
      timerAlarmEnable(timer);
      cur_speed_step = cur_speed_step + 1;
    }else
    {
      timerAlarmDisable(timer);
    }
      
  }

 
  portEXIT_CRITICAL_ISR(&timerMux);
}
