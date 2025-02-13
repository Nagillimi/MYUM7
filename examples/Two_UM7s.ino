/* Example Arduino code for two UM7s sending IMU data at 255 Hz
 *  
 *  - Reads processed gyro, accel, and magnetometer data
 *  - Reads euler data
 * 
 */
#include <MYUM7.h>

// Setup two UM7s on Serial1,2,3
MYUM7 imu1(Serial1); // Thigh
MYUM7 imu2(Serial2); // Shank

long init;

void setup() {
  // Set baud rate between PC and MCU
  Serial.begin(115200);
  
  // Set baud rates on Serial1,2,3 to read at 115200 bps for the MCU
  Serial1.begin(115200);
  Serial2.begin(115200);

  delay(100);

  // Set the UM7s baud rate to output at 115200 bps to match the MCU
  imu1.set_sensor_baud_rate(115200);
  imu2.set_sensor_baud_rate(115200);

  // Set euler and all processed datasets
  imu1.set_euler_rate(255);
  imu2.set_euler_rate(255);
  
  imu1.set_all_processed_rate(255);
  imu2.set_all_processed_rate(255);

  // Zero Gyros and Calibrate Accelerometers
  // imu1.zero_gyros();
  // imu2.zero_gyros();
  // delay(100);
  
  // imu1.calibrate_accelerometers();
  // imu2.calibrate_accelerometers();
  // delay(100);

  init = micros();
}

void loop() {
  Serial.print(micros() - init); Serial.print(", ");
  
  if (Serial1.available() && imu1.decode(Serial1.read())) {
    Serial.print(imu1.roll);
    Serial.print(", ");
    Serial.print(imu1.pitch);
    Serial.print(", ");
    Serial.print(imu1.yaw);
  }
  
  if (Serial2.available() && imu2.decode(Serial2.read())) {
    Serial.print(", ");
    Serial.print(imu2.roll);
    Serial.print(", ");
    Serial.print(imu2.pitch);
    Serial.print(", ");
    Serial.print(imu2.yaw);
  }
  
  Serial.println();
}
