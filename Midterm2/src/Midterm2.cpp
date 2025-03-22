/* 
 * Project Midterm2
 * Author: Efrain Villa
 * Date: 3.20.25
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentials.h"

#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"

#include "Adafruit_BME280.h"

//decleare variables for dashboard and water pump relay
TCPClient TheClient; 

Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

Adafruit_MQTT_Subscribe subFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ButtonOnOff");
Adafruit_MQTT_Subscribe subFeed2 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Slider");
Adafruit_MQTT_Publish pubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/RandomNumber");

unsigned int last, lastTime, lastTime2, lastTime3, lastTime4;
float subValue,pubValue, subValue2;

const int PUMPPIN = D16;
bool buttonValue;

void MQTT_connect();
bool MQTT_ping();

//declare variables for soil sensor OLED interface
const int MOISTPIN = D14;
int currentTime;
int lastSecond;
int moistValue;
const int OLED_RESET = -1;

String dateTime, timeOnly, dateOnly, dateDate, yearOnly;
Adafruit_SSD1306 display(OLED_RESET);

//declare variables used in BME operation
Adafruit_BME280 bme;

int const hexAddress = 0x76;
byte status;
float tempC;
float pressPA;
float humidRH;
float tempF;
float pressHg;
const char deg = 167;

//declare variable for air quality sensor
#include "Air_Quality_Sensor.h"

AirQualitySensor sensor(A0);

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

void setup() {

  pinMode(PUMPPIN, OUTPUT);
  pinMode (MOISTPIN, INPUT);

  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(2000);
  display.clearDisplay();

  Time.zone (-6); 
  Particle.syncTime ();

  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
  Serial.printf(".");
  }
  Serial.printf("\n\n");

    // Setup MQTT subscription
    mqtt.subscribe(&subFeed);
    mqtt.subscribe(&subFeed2);
    
    status = bme.begin (hexAddress);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {

  MQTT_connect();
  MQTT_ping();

  tempC = bme.readTemperature();
  pressPA = bme.readPressure ();
  humidRH = bme.readHumidity ();
  currentTime = millis();
  tempF = tempC * 1.8 +32;
  pressHg = pressPA * .0002953;

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &subFeed) {
      subValue = atof((char *)subFeed.lastread);
      Serial.printf ("%f\n", subValue);
      if(subValue == 1){
        digitalWrite(PUMPPIN, HIGH);
      }
      else{
        digitalWrite(PUMPPIN, LOW);
        if (subscription == &subFeed2) {
          subValue2 = atof((char *)subFeed2.lastread);
          Serial.printf ("%f\n", subValue2);
        }
      }
    }
  }
}

void MQTT_connect() {

  int8_t ret;
  dateTime = Time.timeStr();
  timeOnly = dateTime.substring (11,19);
  dateOnly = dateTime.substring (0,9);
  yearOnly = dateTime.substring (20,24);

  if(millis()-lastTime>10000) {
    lastTime = millis();
  }

  if((millis()-lastTime > 1000)) {
    moistValue = analogRead(MOISTPIN);  
    int quality = sensor.slope();

    Serial.print("Sensor value: ");
    Serial.println(sensor.getValue());
    Serial.printf("moistValue = %i\n", moistValue);
    Serial.printf("Date is: %s", dateOnly.c_str());
    Serial.printf(", %s\n", yearOnly.c_str());
    Serial.printf("Time is %s\n\n", timeOnly.c_str());
    Serial.printf("Temp: %0.0f %cF\nPressure: %0.2f\nHumidity: %0.2f\n", tempF, deg, pressHg, humidRH);
    if (quality == AirQualitySensor::FORCE_SIGNAL) {
      Serial.println("High pollution! Force signal active.");
    } else if (quality == AirQualitySensor::HIGH_POLLUTION) {
      Serial.println("High pollution!");
    } else if (quality == AirQualitySensor::LOW_POLLUTION) {
      Serial.println("Low pollution!");
    } else if (quality == AirQualitySensor::FRESH_AIR) {
      Serial.println("Fresh air.");
    }
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.printf("Soil moisture: %i\n", moistValue);
    display.printf("Date: %s", dateOnly.c_str());
    display.printf(", %s\n", yearOnly.c_str());
    display.printf("Time: %s\n", timeOnly.c_str());
    display.printf("Temp: %0.0f %cF\nPres: %0.0f  Hum: %0.0f\n", tempF, deg, pressHg, humidRH);
    if (quality == AirQualitySensor::FORCE_SIGNAL) {
      display.printf("High pollution! Force signal active.");
    } else if (quality == AirQualitySensor::HIGH_POLLUTION) {
      display.printf("High pollution!");
    } else if (quality == AirQualitySensor::LOW_POLLUTION) {
      display.printf("Low pollution!");
    } else if (quality == AirQualitySensor::FRESH_AIR) {
      display.printf("Fresh air.");
    }
    display.display();
    display.clearDisplay();
    lastTime = millis(); 
  }

    if((millis()-lastTime2 > 6000)) {
    if(mqtt.Update()) {
      pubFeed.publish(tempF);
      } 
    lastTime2 = millis();
    }
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
        if(!pingStatus) {
          Serial.printf("Disconnecting \n");
          mqtt.disconnect();
        }
      last = millis();
    }
  return pingStatus;

if(millis()-lastTime3 > 108000000) {
  if(moistValue>2700) {
   
      digitalWrite(PUMPPIN, HIGH);
      delay(1000);
      digitalWrite(PUMPPIN, LOW);
 
  }
  lastTime3 = millis();
}
}