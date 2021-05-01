/***************************************************************************
 * 
 * Example: Quaternion Animation
 * 
 * This example enables only the quaternion output and sets it to the 
 * maximum output frequency. The output is dumped out to the USB serial port
 * and is used to rotate a 3D object in real-time.
 * 
 ***************************************************************************/

#include <Teensy-ICM-20948.h>

TeensyICM20948 icm20948;

TeensyICM20948Settings icmSettings =
{
  .cs_pin = 10,                   // SPI chip select pin
  .spi_speed = 7000000,           // SPI clock speed in Hz, max speed is 7MHz
  .mode = 1,                      // 0 = low power mode, 1 = high performance mode
  .enable_gyroscope = false,      // Enables gyroscope output
  .enable_accelerometer = false,  // Enables accelerometer output
  .enable_magnetometer = false,   // Enables magnetometer output
  .enable_quaternion = true,      // Enables quaternion output
  .gyroscope_frequency = 1,       // Max frequency = 225, min frequency = 1
  .accelerometer_frequency = 1,   // Max frequency = 225, min frequency = 1
  .magnetometer_frequency = 1,    // Max frequency = 70, min frequency = 1
  .quaternion_frequency = 225     // Max frequency = 225, min frequency = 50
};

void setup()
{
  Serial.begin(115200);
  delay(5000);

  icm20948.init(icmSettings);
}

void loop()
{
  float quat_w, quat_x, quat_y, quat_z;
  char sensor_string_buff[128];

  // Must call this often in main loop -- updates the sensor values
  icm20948.task();

  if (icm20948.quatDataIsReady())
  {
    icm20948.readQuatData(&quat_w, &quat_x, &quat_y, &quat_z);
    
    // Store data in a JSON string and send it over the serial port
    sprintf(sensor_string_buff, "{\"quat_w\":%f, \"quat_x\":%f, \"quat_y\":%f, \"quat_z\":%f}", quat_w, quat_x, quat_y, quat_z);
    Serial.println(sensor_string_buff);
  }
}
