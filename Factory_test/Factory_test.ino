#include <TFT_eSPI.h>
#include <TFT_Touch.h>
#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2cScd30.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#include <SoftwareSerial.h>
SoftwareSerial mySerial(16, 17); // RX, TX
unsigned int pm1 = 0;
unsigned int pm2_5 = 0;
unsigned int pm10 = 0;

static char errorMessage[128];
static int16_t error;

float co2Concentration = 0.0;
float temperature = 0.0;
float humidity = 0.0;

SensirionI2cScd30 sensor;

TFT_eSPI tft = TFT_eSPI(240, 320); /* TFT instance */

// These are the pins used to interface between the 2046 touch controller and Arduino Pro
#define DOUT 39  /* Data out pin (T_DO) of touch screen */
#define DIN  32  /* Data in pin (T_DIN) of touch screen */
#define DCS  33  /* Chip select pin (T_CS) of touch screen */
#define DCLK 25  /* Clock pin (T_CLK) of touch screen */

/* Create an instance of the touch screen library */
TFT_Touch touch = TFT_Touch(DCS, DCLK, DIN, DOUT);

TaskHandle_t Task1;
void TaskAnalogRead( void *pvParameters );

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  mySerial.begin(9600);
  Wire.begin(27, 22);
  sensor.begin(Wire, SCD30_I2C_ADDR_61);

  sensor.stopPeriodicMeasurement();
  sensor.softReset();
  delay(2000);
  uint8_t major = 0;
  uint8_t minor = 0;
  error = sensor.readFirmwareVersion(major, minor);
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute readFirmwareVersion(): ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }
  Serial.print("firmware version major: ");
  Serial.print(major);
  Serial.print("\t");
  Serial.print("minor: ");
  Serial.print(minor);
  Serial.println();
  error = sensor.startPeriodicMeasurement(0);
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }

  tft.begin();          /* TFT init */
  tft.setRotation(1); /* Landscape orientation, flipped */
  touch.setCal(526, 3443, 750, 3377, 320, 240, 1);
  tft.fillScreen(0x0);

  // Display Menu
  tft.setCursor(15, 20); // Set the cursor position X,Y
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE);
  tft.print("DUST BOY Test");

  tft.setCursor(50, 90); // Set the cursor position X,Y
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE);
  tft.print("PM2.5");

  tft.setCursor(50, 125); // Set the cursor position X,Y
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE);
  tft.print("PM10");

  tft.setCursor(50, 165); // Set the cursor position X,Y
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE);
  tft.print("Temp");
  tft.setCursor(245, 165); // Set the cursor position X,Y
  tft.print("C");

  tft.setCursor(50, 205); // Set the cursor position X,Y
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE);
  tft.print("Humi");
  tft.setCursor(245, 205); // Set the cursor position X,Y
  tft.print("%");

  tft.setCursor(50, 50); // Set the cursor position X,Y
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE);
  tft.print("CO2");

  // This variant of task creation can also specify on which core it will be run (only relevant for multi-core ESPs)
  xTaskCreatePinnedToCore(
    TaskAnalogRead,   /* Task function. */
    "Analog Read",     /* name of task. */
    20000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task1,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
}

/* Main program */
void loop()
{
  delay(1000);

  Serial.print("Co2:");
  Serial.print(co2Concentration);
  Serial.print("\t");
  Serial.print("Temperature:");
  Serial.print(temperature);
  Serial.print("\t");
  Serial.print("Humidity:");
  Serial.println(humidity);

}


void TaskAnalogRead(void *pvParameters) { // This is a task.
  (void) pvParameters;
  // Check if the given analog pin is usable - if not - delete this task

  for (;;) {

    tft.setTextSize(3);
    tft.setCursor(150, 50); // Set the cursor position X,Y
    tft.setTextColor(TFT_BLACK);
    tft.println(co2Concentration);
    // tft.setTextFont(GLCD);
    tft.setTextSize(3);
    tft.setCursor(150, 85);
    tft.setTextColor(TFT_BLACK);
    tft.println(pm2_5);//massConcentrationPm2p5
    // tft.setTextFont(GLCD);
    tft.setTextSize(3);
    tft.setCursor(150, 120);
    tft.setTextColor(TFT_BLACK);
    tft.println(pm10);//massConcentrationPm10p0
    //
    tft.setTextSize(3);
    tft.setCursor(150, 160);
    tft.setTextColor(TFT_BLACK);
    tft.println(temperature);
    // tft.setTextFont(GLCD);
    tft.setTextSize(3);
    tft.setCursor(150, 200);
    tft.setTextColor(TFT_BLACK);
    tft.println(humidity);

    error = sensor.blockingReadMeasurementData(co2Concentration, temperature,
            humidity);

    int index = 0;
    char value;
    char previousValue;


    while (mySerial.available()) {
      value = mySerial.read();
      if ((index == 0 && value != 0x42) || (index == 1 && value != 0x4d)) {
        Serial.println("Cannot find the data header.");
        break;
      }

      if (index == 4 || index == 6 || index == 8 || index == 10 || index == 12 || index == 14) {
        previousValue = value;
      }
      else if (index == 5) {
        pm1 = 256 * previousValue + value;
        Serial.print("{ ");
        Serial.print("\"pm1\": ");
        Serial.print(pm1);
        Serial.print(" ug/m3");
        Serial.print(", ");
      }
      else if (index == 7) {
        pm2_5 = 256 * previousValue + value;
        Serial.print("\"pm2_5\": ");
        Serial.print(pm2_5);
        Serial.print(" ug/m3");
        Serial.print(", ");
      }
      else if (index == 9) {
        pm10 = 256 * previousValue + value;
        Serial.print("\"pm10\": ");
        Serial.print(pm10);
        Serial.print(" ug/m3");
      } else if (index > 15) {
        break;
      }
      index++;
    }

    tft.setTextSize(3);
    tft.setCursor(150, 50); // Set the cursor position X,Y
    tft.setTextColor(TFT_PINK);
    tft.println(co2Concentration);

    tft.setTextSize(3);
    tft.setCursor(150, 85);
    tft.setTextColor(TFT_ORANGE);
    tft.println(pm2_5);//massConcentrationPm2p5
    // tft.setTextFont(GLCD);
    tft.setTextSize(3);
    tft.setCursor(150, 120);
    tft.setTextColor(TFT_ORANGE);
    tft.println(pm10);//massConcentrationPm10p0
    //
    // tft.setTextFont(GLCD);
    tft.setTextSize(3);
    tft.setCursor(150, 160);
    tft.setTextColor(TFT_RED);
    tft.println(temperature);
    // tft.setTextFont(GLCD);
    tft.setTextSize(3);
    tft.setCursor(150, 200);
    tft.setTextColor(TFT_GREEN);
    tft.println(humidity);
    //
    delay(1000);
    //
    //        // tft.setTextFont(GLCD);
    //        tft.setTextSize(4);
    //        tft.setCursor(150, 120);
    //        tft.setTextColor(TFT_BLACK);
    //        tft.println(average1);
  }
}
