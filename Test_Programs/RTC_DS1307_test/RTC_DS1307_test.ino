/*
The DS1307 communicates using the I2C protocol. You'll need to connect the SDA and SCL pins of the RTC to the corresponding I2C pins on the ESP32.

Connections:
VCC (DS1307) to 3.3V (ESP32)
GND (DS1307) to GND (ESP32)
SDA (DS1307) to SDA (ESP32 GPIO 21) - This might vary depending on your board.
SCL (DS1307) to SCL (ESP32 GPIO 22) - This might vary depending on your board.
Required Libraries
You will need the Wire library for I2C communication and the RTClib library for interacting with the DS1307. You can install these through the Arduino Library Manager.
*/

#include <Wire.h>
#include <RTClib.h>

RTC_DS1307 rtc;

void setup() {
  Serial.begin(115200);

  // Initialize I2C communication
  Wire.begin(27, 22);

  // Check if the RTC is connected properly
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  // Check if the RTC lost power and if so, set the time
  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // This line sets the RTC with an explicit date & time, for example to set
    // January 1, 2024 at 00:00 you would call:
    rtc.adjust(DateTime(2024, 1, 1, 0, 0, 0));
  }
}

void loop() {
  DateTime now = rtc.now();

  // Print the current date and time to the serial monitor
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  delay(1000);  // Wait for 1 second
}
