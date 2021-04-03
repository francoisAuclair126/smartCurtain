#include <Adafruit_NeoPixel.h>
#include "SdsDustSensor.h"


#define LED_PIN 9
#define enable 3
#define motorDir1 7
#define motorDir2 6
#define light A0

#define LED_COUNT 2

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define BRIGHTNESS 20 // Set BRIGHTNESS to about 1/5 (max = 255)


// using StateDiagram logic
enum State {
  opened,
  closed,
  opening,
  closing
};

// REED SWITCHES + LIGHT
const int reedOpened = 1; // Pin connected to reed switch
const int reedClosed = 2; // Pin connected to reed switch
const int lightTreshold = -1; //TODO: SET TO 300

//SDS011
const int airRxPin = 10; // txSensorPin
const int airTxPin = 11; // rxSensorPin
const int numAirReadings = 5; //numAirReadings is the number of data that it will be taken to creat a average data (smoothing the data)
const float airThresholdVariation = 5;
const int numChannel = 2;

//TEMPERATURE SENSOR
const int temperaturePin = A1;


//Timing
unsigned long ledShowTime = 0; // 50 days max


float readings[numChannel][numAirReadings];
float runningAverage[numChannel];
float total[numChannel];
int readingIndex = 0;
SdsDustSensor sds(airRxPin, airTxPin);

State currentState;

void setup() {
  pinMode(enable, OUTPUT);
  pinMode(motorDir1, OUTPUT);
  pinMode(motorDir2, OUTPUT);
  pinMode(light, INPUT);
  pinMode(temperaturePin, INPUT);

  pinMode(reedOpened, INPUT_PULLUP);
  pinMode(reedClosed, INPUT_PULLUP);

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(BRIGHTNESS);

  Serial.begin(9600);
  delay(1);
  sds.begin(); //sensor be prepare to work
  delay(1);

  Serial.println(sds.queryFirmwareVersion().toString()); // prints firmware version
  delay(1);
  Serial.println(sds.setActiveReportingMode().toString()); // ensures sensor is in 'active' reporting mode
  delay(1);
  Serial.println(sds.setContinuousWorkingPeriod().toString()); // sensor sends data every 3 minutes
  delay(1);

  //initializing variables for air sensor
  for (int channel = 0; channel < numChannel; channel++) {
    runningAverage[channel] = 0;
    total[channel] = 0;
    for (int reading = 0; reading <= numAirReadings; reading++) {
      readings[channel][reading] = 0;
    }
  }

  currentState = opening;
  setDirBreak();

  // speed won't change so why not
  int pwmOutput = 255;
  analogWrite(enable, pwmOutput); // Send PWM signal to L298N enableble pin

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  currentState = closed; //TODO: REMOVE
  switch (currentState) {
    case opening:
      if (digitalRead(reedOpened) == LOW) {
        setDirBreak();
        currentState = opened;
      }
      else {
        setDirOpening();
      }
      break;
    case opened:
      if (analogRead(light) > lightTreshold) {
        setDirClosing();
        currentState = closing;
      }
      break;
    case closing:
      if (digitalRead(reedClosed) == LOW) {
        setDirBreak();
        currentState = closed;
        ledShowTime = millis();
      }
      break;
    case closed:
      if (analogRead(light) < lightTreshold) {
        setDirOpening();
        currentState = opening;
      } else {
        unsigned long currentTime = millis();
        if ((currentTime - ledShowTime) % 1000 == 0) {
          float* currentAirQuality = monitorAirQuality();
          int currentTemperature = 200 ;//analogRead(temperaturePin);
          setStripColor(currentTemperature);
        }
      }
      break;
  }

}

// Utility functions to set motor directions
void setDirOpening() {
  digitalWrite(motorDir1, LOW);
  digitalWrite(motorDir2, HIGH);
  delay (1);
}

void setDirClosing() {
  digitalWrite(motorDir1, HIGH);
  digitalWrite(motorDir2, LOW);
  delay (1);
}

void setDirBreak() {
  digitalWrite(motorDir1, LOW);
  digitalWrite(motorDir2, LOW);
  delay (1);
}

float* monitorAirQuality() {
  //read data
  PmResult pm = sds.readPm();

  if (pm.isOk()) {
    for (int channel = 0 ; channel < numChannel ; channel++) {
      // subtract oldest reading
      total[channel] = total[channel] - readings[channel][readingIndex];

      if (channel == 0) {
        Serial.println("---CHANNEL 25 ---");
        //readingIndex = l'offset de l'index (0-1-2-3-4-5)
        readings[channel][readingIndex] = pm.pm25;
      } else if (channel == 1) {
        Serial.println("--- CHANNEL 10 ---");
        readings[channel][readingIndex] = pm.pm10;
      }

      //update totals
      total[channel] += readings[channel][readingIndex];

      //update average
      runningAverage[channel] = total[channel] / numAirReadings;
      Serial.println(runningAverage[channel]);
      delay(1);
    }

    //update index
    readingIndex++;
    if (readingIndex >= numAirReadings) {
      readingIndex = 0;
    }
  } else {
    //          Serial.println("Could not read values from sensor, reason: ");
    //          delay(1);
    //          Serial.println(pm.statusToString());
    //          delay(1);
  }
  return runningAverage;
}


void setStripColor (int temperature) {

  uint16_t mappedTemperature = map(temperature, 300, 700, 32768, 0);
  uint32_t newColor = strip.ColorHSV(constrain(mappedTemperature,0,32768));
  strip.fill(newColor);
  strip.show();
}
// dont use setBrightness: https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-library-use
void setStripBrightness(float air25, float air10) { 


}
