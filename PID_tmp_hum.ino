
// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

#include "DHT.h"
#include <Phant.h>
#include <pgmspace.h>
#include <PID_v1.h>


// rely control pins
#define HUMID_RELAY_PIN 5
#define HEAT_RELAY_PIN 6

#define LIGHT_SENSE_PIN A0
#define DHTPIN 3     // what digital pin Tmp/Humid sensor is connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

// Arduino example stream
// http://data.sparkfun.com/streams/VGb2Y1jD4VIxjX3x196z
// host, public key, private key
Phant phant("data.sparkfun.com", "5JmnajMjGXud489r1ORN", "7BGb5gngqacqvBJ7Prjm");

// PID controllers init

//Define Variables we'll be connecting to
double SetpointT, InputT, OutputT;
double SetpointH, InputH, OutputH;

//Specify the links and initial tuning parameters
double KpT=2, KiT=5, KdT=1;
PID tmpPID(&InputT, &OutputT, &SetpointT, KpT, KiT, KdT, DIRECT);
double KpH=2, KiH=5, KdH=1;
PID humidPID(&InputH, &OutputH, &SetpointH, KpH, KiH, KdH, DIRECT);

// PID discretization 
int WindowSize = 5000;
unsigned long windowStartTime;

int print_rate = 5000;
unsigned long last_print;

void setup()
{
  Serial.begin(115200);
  Serial.println("Hello there!");

  dht.begin();
  
  windowStartTime = millis();
  last_print = millis();

  //initialize the variables we're linked to
  SetpointT = 65;
  SetpointH = 90;

  //tell the PID to range between 0 and the full window size
  tmpPID.SetOutputLimits(0, WindowSize);
  humidPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  tmpPID.SetMode(AUTOMATIC);
  humidPID.SetMode(AUTOMATIC);
}

void loop()
{
  // process PIDs
  tmpPID.Compute();
  humidPID.Compute();
  
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Fahrenheit (isFahrenheit = true else celsuis)
  float f = dht.readTemperature(true);
  // light level
  float l = map(analogRead(LIGHT_SENSE_PIN), 0, 1023, 0, 10000) / 100;
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    delay(1000);
    return;
  } else {
    InputT = f;
    InputH = h;
    if (millis() - last_print > print_rate) {
      Serial.print("Humidity: ");
      Serial.print(h);
      Serial.print(" %\t");
      Serial.print("Temperature: ");
      Serial.print(f);
      Serial.println(" *F\t");
      Serial.print("PID Temp Out:");
      Serial.print(OutputT);
      Serial.println(" \t");
      Serial.print("PID Humid Out:");
      Serial.print(OutputH);
      Serial.println(" \t");
      
      // build phant post
      phant.add("humidity", String(h));
      phant.add("temp", String(f));
      phant.add("light", String(l));
      phant.add("pid_t", String(OutputT));
      phant.add("pid_h", String(OutputH));
      Serial.println("----HTTP POST----");
      Serial.println(phant.post());
      
      // send to phant
      
      
      // updte timer
      last_print = millis();
    }
  }

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  // temp ctrl
  if (OutputT < millis() - windowStartTime) digitalWrite(HEAT_RELAY_PIN, HIGH);
  else digitalWrite(HEAT_RELAY_PIN, LOW);
  // humid ctrl
  if (OutputH < millis() - windowStartTime) digitalWrite(HUMID_RELAY_PIN, HIGH);
  else digitalWrite(HUMID_RELAY_PIN, LOW);

}



