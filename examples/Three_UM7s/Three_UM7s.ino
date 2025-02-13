/* Example Arduino code for one UM7 sending IMU data at 255 Hz
 *  
 *  - Reads processed gyro, accel, and magnetometer data
 *  - Reads euler data
 * 
 */
#include <MYUM7.h>

// Setup three UM7s on Serial1,2,3
MYUM7 imu1(Serial1); // Thigh
MYUM7 imu2(Serial2); // Shank
MYUM7 imu3(Serial3); // Foot

void setup() {
  // Set baud rate between PC and MCU
  Serial.begin(115200);
  
  // Set baud rates on Serial1,2,3 to read at 115200 bps for the MCU
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);

  delay(1000);

  // Set the UM7s baud rate to output at 115200 bps to match the MCU
  imu1.set_sensor_baud_rate(115200);
  imu2.set_sensor_baud_rate(115200);
  imu3.set_sensor_baud_rate(115200);

  // Set euler and all processed datasets
  imu1.set_euler_rate(255);
  imu2.set_euler_rate(255);
  imu3.set_euler_rate(255);
  
  imu1.set_all_processed_rate(255);
  imu2.set_all_processed_rate(255);
  imu3.set_all_processed_rate(255);

  // Zero Gyros and Calibrate Accelerometers
  // Note: Calibration sets the biasing for the XYZ accel vectors, it will rotate the coordinate system to center about [0,0,1](G).
//   imu1.zero_gyros();
//   imu2.zero_gyros();
//   imu3.zero_gyros();
//   delay(2000);
//   imu1.calibrate_accelerometers();
//   imu2.calibrate_accelerometers();
//   imu3.calibrate_accelerometers();
//   delay(2000);
}

void loop() {
    
  if (Serial1.available() > 0 && Serial2.available() > 0 && Serial3.available() > 0) {   
    
    // Reads byte from buffer, valid packet returns true and executes if statement.
    // if (imu1.decode(Serial1.read()) && imu2.decode(Serial2.read()) && imu3.decode(Serial3.read())) {
    if (imu1.decode(Serial1.read()) || imu2.decode(Serial2.read()) || imu3.decode(Serial3.read())) {

      // Print euler datasets from each UM7     
      Serial.print(imu1.roll); Serial.print(", ");
      Serial.print(imu1.pitch); Serial.print(", ");
      Serial.print(imu1.yaw); Serial.print(", ");

      Serial.print(imu2.roll); Serial.print(", ");
      Serial.print(imu2.pitch); Serial.print(", ");
      Serial.print(imu2.yaw); Serial.print(", ");

      Serial.print(imu3.roll); Serial.print(", ");
      Serial.print(imu3.pitch); Serial.print(", ");
      Serial.println(imu3.yaw);
      
      Serial.print("\n");
     }
  }
}
