/***************************************************************************
 * 
 * Example: Read Sensor Values
 * 
 * This example grabs the values from all sensors and spits them out over
 * the serial port. I've set all of the sensors to update at their minimum
 * frequency. You'll notice there are many more quaternion readings compared
 * to the others -- this is because the minium quaternion output frequency
 * is 50Hz, while the minimum for the other sensors is 1Hz.
 * 
 ***************************************************************************/

#include <Teensy-ICM-20948.h>

TeensyICM20948 icm20948;

TeensyICM20948Settings icmSettings =
{
  .cs_pin = 10,                  // SPI chip select pin
  .spi_speed = 7000000,          // SPI clock speed in Hz, max speed is 7MHz
  .mode = 1,                     // 0 = low power mode, 1 = high performance mode
  .enable_gyroscope = true,      // Enables gyroscope output
  .enable_accelerometer = true,  // Enables accelerometer output
  .enable_magnetometer = true,   // Enables magnetometer output
  .enable_quaternion = true,     // Enables quaternion output
  .gyroscope_frequency = 1,      // Max frequency = 225, min frequency = 1
  .accelerometer_frequency = 1,  // Max frequency = 225, min frequency = 1
  .magnetometer_frequency = 1,   // Max frequency = 70, min frequency = 1
  .quaternion_frequency = 50     // Max frequency = 225, min frequency = 50
};

void setup()
{
  Serial.begin(115200);
  delay(5000);

  icm20948.init(icmSettings);
}

void loop()
{
  float gyro_x, gyro_y, gyro_z;
  float accel_x, accel_y, accel_z;
  float mag_x, mag_y, mag_z;
  float quat_w, quat_x, quat_y, quat_z;
  char sensor_string_buff[128];

  // Must call this often in main loop -- updates the sensor values
  icm20948.task();

  if (icm20948.gyroDataIsReady())
  {
    icm20948.readGyroData(&gyro_x, &gyro_y, &gyro_z);
    sprintf(sensor_string_buff, "Gyro (deg/s): [%f,%f,%f]", gyro_x, gyro_y, gyro_z);
    Serial.println(sensor_string_buff);
  }

  if (icm20948.accelDataIsReady())
  {
    icm20948.readAccelData(&accel_x, &accel_y, &accel_z);
    sprintf(sensor_string_buff, "Accel (g): [%f,%f,%f]", accel_x, accel_y, accel_z);
    Serial.println(sensor_string_buff);
  }

  if (icm20948.magDataIsReady())
  {
    icm20948.readMagData(&mag_x, &mag_y, &mag_z);
    sprintf(sensor_string_buff, "Mag (uT): [%f,%f,%f]", mag_x, mag_y, mag_z);
    Serial.println(sensor_string_buff);
  }

  if (icm20948.quatDataIsReady())
  {
    icm20948.readQuatData(&quat_w, &quat_x, &quat_y, &quat_z);
    sprintf(sensor_string_buff, "Quat (deg): [%f,%f,%f,%f]", quat_w, quat_x, quat_y, quat_z);
    Serial.println(sensor_string_buff);
  }
}
