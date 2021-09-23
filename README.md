# RedShift Lab's UM7 Driver for Arduino IDE
[UM7 Orientation Sensor](https://redshiftlabs.com.au/product/um7-orientation-sensor/)

*Revision 2: June 8, 2020:*
- Allows for configurable communication with x UM7 units on a given x UART bus.
- Contains all configuration and command registers
- Can read from all data registers
- Can send firmware and error code

*Important Notes:*
- I no longer have access to these sensors, so don't hesitate to put a PR in given you've done some tests etc. etc.
- This driver is inherently synchronous (stream-only) and requires overhead from the Arduino's Serial
- The UM7 sensor does **not** have an interrupt pin (hence the shift to the [SPI bus](https://github.com/Nagillimi/MYUM7SPI))

## Accesible Variables
*Functional Variables*
```c++
HardwareSerial& serial_port;
uint32_t 	error;
```
*Raw Variables*
```c++
int16_t 	gyro_raw_x, gyro_raw_y, gyro_raw_z;
int16_t 	accel_raw_x, accel_raw_y, accel_raw_z;
int16_t 	mag_raw_x, mag_raw_y, mag_raw_z;
float 		temp, temp_time;
float 		gyro_raw_time, accel_raw_time, mag_raw_time;
```
*Processed Variables*
```c++
float 		gyro_x, gyro_y, gyro_z, gyro_time;
float 		accel_x, accel_y, accel_z, accel_time;
float 		mag_x, mag_y, mag_z, mag_time;
```
*Orientation Variables*
```c++
int16_t 	quat_a, quat_b, quat_c, quat_d, quat_time;
int16_t 	roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate;
float 		euler_time;
```
*Pose/Velocity Variables*
```c++
float 		north_pos, east_pos, up_pos, pos_time;
float 		north_vel, east_vel, up_vel, vel_time;
```
*GPS/SAT Variables*
```c++
float 		lattitude, longitude, altitude, course, speed, gps_time;
float 		satellite_id[12], satellite_SNR[12];
```
*Gyro Bias Variables*
```c++
float 		gyro_bias_x, gyro_bias_y, gyro_bias_z;
```
## Accesible Functions
```c++
decode(byte current_byte)
```
The function that decodes and parses incoming data from the UM7.  
The function will determine which register is being read, the batch length, and call checksum once finished.  
current_byte is current byte being read by serial.  
decode() returns true if a packet was read succesfully (if checksum returns true).  
Have the argument for "decode" SerialX.read() where X is the initialized Serial associated with the UM7  
***
```c++
error_status()
```
Parses error into string outputs  
***
```c++
set_sensor_baud_rate(float baud)
```
Sets the UM7 baud rate. If not called, default is 115200bps  
***
```c++
set_sensor_baud_rate(float baud, float gps_baud, bool gps, bool sat)
```
Overloaded function for set_sensor_baud_rate. Parameters are the UM7 baud rate as well as the gps baud rate.
Combines into one register (1B) for command.
GPS bit = Causes GPS data to be transmitted automatically once received externally, stored as a batch from DREG_GPS_LATITUDE
SAT bit = Causes SAT data to be transmitted automatically once received externally, stored as batch DREG_GPS_SAT_1_2  
***
```c++
set_raw_rate(uint8_t accel_rate, uint8_t gyro_rate, uint8_t mag_rate)
```
Sets the individual rates for the raw datasets. 
From 0 to 255 Hz, sent through their respective in their signed bytes  
***
```c++
set_all_raw_rate(uint8_t rate)
```
Sets the rate for all raw datasets to the same desired rate  
***
```c++
set_all_raw_rate(uint8_t temp_rate, uint8_t rate)
```
Overloaded function for set_all_raw_rate. Allows for the temperature rate to be altered without defaulting the raw datarate
Sets the rate for the temperature datasets as well as raw datasets.
Leave "rate" as zero if you'd like to keep the raw data rate at a default (or preset individual) speed(s)  
***
```c++
set_processed_rate(uint8_t accel_rate, uint8_t gyro_rate, uint8_t mag_rate)
```
Sets the individual rates for the processed datasets. 
From 0 to 255 Hz, sent through their respective in their signed bytes.
***
```c++
set_all_processed_rate(uint8_t rate)
```
Sets the rate for all processed datasets to the same desired rate
***
```c++
set_quaternion_rate(uint8_t rate)
```
Sets the rate for all quaternion datasets
***
```c++
set_euler_rate(uint8_t rate)
```
Sets the rate for all euler angle datasets
***
```c++
set_position_rate(uint8_t rate)
```
Sets the rate for all position datasets
***
```c++
set_velocity_rate(uint8_t rate)
```
Sets the rate for all velocity datasets
***
```c++
set_pose_rate(uint8_t rate)
```
Sets the rate for all position and euler angle datasets to the same desired rate
***
```c++
set_health_rate(float baud)
```
Sets the rate for the health packet. Default is 1 Hz.
***
```c++
set_gyro_bias_rate(uint8_t rate)
```
Sets the rate for all gyro bias datasets
***
```c++
set_NMEA_health_rate(int8_t baud)
```
Sets the rate for the NMEA health packet
***
```c++
set_NMEA_pose_rate(int8_t baud)
```
Sets the rate for the NMEA position and euler angle datasets
***
```c++
set_NMEA_attitude_rate(int8_t baud)
```
Sets the rate for the NMEA attitude datasets
***
```c++
set_NMEA_rates_rate(int8_t baud)
```
Sets the rate for the NMEA rates datasets
***
```c++
set_NMEA_GPS_pose_rate(int8_t baud)
```
Sets the rate for the NMEA GPS pose datasets
***
```c++
set_NMEA_quaternion_rate(int8_t baud)
```
Sets the rate for the NMEA quaternion datasets
***
```c++
set_misc_ssettings(bool pps, bool zg, bool q, bool mag)
```
Miscellaneous settings for filter and sensor control options. Send a 0 if you don't wish to configure a specific setting
Ex. set_misc_settings(0, 1, 1, 0)
PPS bit = Causes the TX2/RX2 pin to be used with an external GPS
ZG bit = Causes UM7 to measure gyro bias at setup
Q bit = Sensor will run in Quternion mode instead of Euler mode. Fixes pitch error in the Gimbal lock position
MAG bit = Magnetometer will be used in state updates
***
```c++
set_home_north(float north)
```
Configuration for hard setting the north orientation vector
***
```c++
set_home_east(float east)
```
Configuration for hard setting the east orientation vector
***
```c++
set_home_up(float up)
```
Configuration for hard setting the up orientation vector
***
```c++
set_gyro_trim(float trim_x, float trim_y, float trim_z)
```
Configuration for the gyro trim in XYZ. It's calculated additioanly to the initial bias compensation (done in 
the ZERO_GYRO_BIAS command).
***
```c++
soft_iron_magnetometer_calibration(float (*array)[3][3])
```
Uses 9 entries to perform a soft-iron calibration of the magnetometer. These terms are computed from the RedShiftLabs
Serial Interface
***
```c++
hard_iron_magnetometer_calibration(float bias_x, float bias_y, float bias_z)
```
Performs a hard-iron calibration of the magnetometer. These terms are computed from the RedShiftLabs
Serial Interface
***
```c++
accelerometer_misalignment_compensation(float (*array)[3][3])
```
Uses 9 entries to the cross axis misalignment of the accelerometer. 
***
```c++
accelerometer_calibration(float bias_x, float bias_y, float bias_z)
```
Performs a calibration for the accelerometer bias in XYZ directions. These terms are computed from the RedShiftLabs
Serial Interface
***
```c++
get_firmware_revision()
```
Causes UM7 to transmit a packet containing the firmware revision string (a 4B char sequence)
***
```c++
save_configs_to_flash()
```
Causes the UM7 to write all configuration settings to FLASH so that they will remain when the power is cycled.
***
```c++
factory_reset()
```
Causes the UM7 to load default factory settings.
***
```c++
zero_gyros()
```
Causes the UM7 to measure the gyro outputs and set the output trim registers to compensate for any non-zero bias. 
The UM7 should be kept stationary while the zero operation is underway.
***
```c++
set_home_position()
```
Sets the current GPS latitude, longitude, and altitude as the home position. 
All future positions will be referenced to the current GPS position.
***
```c++
set_mag_reference()
```
Sets the current yaw heading position as north.
***
```c++
calibrate_accelerometers()
```
Reboots the UM7 and performs a crude calibration on the accelerometers. Best performed on a flat surface.
***
```c++
reset_kalman_filter()
```
Resets the EKF. Extended Kalman Filter (EKF)
***
## Registers Implemented as per the Datasheet
[Datasheet](https://redshiftlabs.com.au/wp-content/uploads/2018/02/um7_datasheet_v1-6_10.1.2016.pdf)
### Configuration Registers
```
CREG_COM_SETTINGS
CREG_COM_RATES1
CREG_COM_RATES2
CREG_COM_RATES3
CREG_COM_RATES4
CREG_COM_RATES5
CREG_COM_RATES6
CREG_COM_RATES7
CREG_MISC_SETTING

CREG_HOME_NORTH
CREG_HOME_EAST
CREG_HOME_UP

CREG_GYRO_TRIM_X 
CREG_GYRO_TRIM_Y 
CREG_GYRO_TRIM_Z 

CREG_MAG_CAL1_1
CREG_MAG_CAL1_2
CREG_MAG_CAL1_3
CREG_MAG_CAL2_1
CREG_MAG_CAL2_2
CREG_MAG_CAL2_3
CREG_MAG_CAL3_1
CREG_MAG_CAL3_2
CREG_MAG_CAL3_3

CREG_MAG_BIAS_X
CREG_MAG_BIAS_Y
CREG_MAG_BIAS_Z

CREG_ACCEL_CAL1_1
CREG_ACCEL_CAL1_2
CREG_ACCEL_CAL1_3
CREG_ACCEL_CAL2_1
CREG_ACCEL_CAL2_2
CREG_ACCEL_CAL2_3
CREG_ACCEL_CAL3_1
CREG_ACCEL_CAL3_2
CREG_ACCEL_CAL3_3

CREG_ACCEL_BIAS_X
CREG_ACCEL_BIAS_Y
CREG_ACCEL_BIAS_Z
```
### Data Registers
```
DREG_HEALTH
DREG_GYRO_RAW_XY
DREG_GYRO_RAW_Z
DREG_GYRO_RAW_TIME
DREG_ACCEL_RAW_XY
DREG_ACCEL_RAW_Z
DREG_ACCEL_RAW_TIME
DREG_MAG_RAW_XY
DREG_MAG_RAW_Z
DREG_MAG_RAW_TIME
DREG_TEMPERATURE
DREG_TEMPERATURE_TIME

DREG_GYRO_PROC_X
DREG_GYRO_PROC_Y
DREG_GYRO_PROC_Z
DREG_GYRO_PROC_TIME
DREG_ACCEL_PROC_X
DREG_ACCEL_PROC_Y
DREG_ACCEL_PROC_Z
DREG_ACCEL_PROC_TIME
DREG_MAG_PROC_X
DREG_MAG_PROC_Y
DREG_MAG_PROC_Z
DREG_MAG_PROC_TIME

DREG_QUAT_AB
DREG_QUAT_CD
DREG_QUAT_TIME
DREG_EULER_PHI_THETA
DREG_EULER_PSI
DREG_EULER_PHI_THETA_DOT
DREG_EULER_PSI_DOT
DREG_EULER_TIME
DREG_POSITION_N
DREG_POSITION_E
DREG_POSITION_UP
DREG_POSITION_TIME
DREG_VELOCITY_N
DREG_VELOCITY_E
DREG_VELOCITY_UP
DREG_VELOCITY_TIME

DREG_GPS_LATITUDE
DREG_GPS_LONGITUDE
DREG_GPS_ALTITUDE
DREG_GPS_COURSE
DREG_GPS_SPEED
DREG_GPS_TIME
DREG_GPS_SAT_1_2
DREG_GPS_SAT_3_4
DREG_GPS_SAT_5_6
DREG_GPS_SAT_7_8
DREG_GPS_SAT_9_10
DREG_GPS_SAT_11_12

DREG_GYRO_BIAS_X
DREG_GYRO_BIAS_Y
DREG_GYRO_BIAS_Z
```
### Command Regsiters
```
GET_FW_REVISION
FLASH_COMMIT
RESET_TO_FACTORY
ZERO_GYROS
SET_HOME_POSITION
SET_MAG_REFERENCE
CALIBRATE_ACCELEROMETERS
RESET_EKF
```
