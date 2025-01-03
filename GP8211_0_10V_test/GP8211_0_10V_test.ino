#include <DFRobot_GP8XXX.h>

DFRobot_GP8211S GP8211S;

void setup() {
  Serial.begin(115200);
  Wire.begin(27, 22);
  while (GP8211S.begin() != 0) {
    Serial.println("Communication with the device has encountered a failure. Please verify the integrity of the connection or ensure that the device address is properly configured.");
    delay(1000);
  }
  GP8211S.setDACOutRange(GP8211S.eOutputRange10V);
  /**
     @brief. Configuring different channel outputs for DAC values
     @param data. Data values corresponding to voltage values
     @n （0 - 32767）.This module is a 15-bit precision DAC module, hence the values ranging from 0 to 32767 correspond to voltages of 0-10V respectively.
  */
  GP8211S.setDACOutVoltage(21456);//Output 6.548V

  delay(5000);

  GP8211S.setDACOutVoltage(32767);//Output 10V
  delay(5000);
  //The set voltage is saved internally in the chip for power-off retention.
  //GP8211S.store();
}

void loop() {

}
