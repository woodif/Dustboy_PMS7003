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

void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char * path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}


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

  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {

    tft.setTextColor(TFT_RED);
    tft.setCursor(245, 80); // Set the cursor position X,Y
    tft.setTextSize(3);
    tft.print("SDHC");

    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  listDir(SD, "/", 0);
  createDir(SD, "/mydir");
  listDir(SD, "/", 0);
  removeDir(SD, "/mydir");
  listDir(SD, "/", 2);
  writeFile(SD, "/hello.txt", "Hello ");
  appendFile(SD, "/hello.txt", "World!\n");
  readFile(SD, "/hello.txt");
  deleteFile(SD, "/foo.txt");
  renameFile(SD, "/hello.txt", "/foo.txt");
  readFile(SD, "/foo.txt");
  testFileIO(SD, "/test.txt");
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));

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
