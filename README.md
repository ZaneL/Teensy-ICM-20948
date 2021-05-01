
# Arduino library for the ICM-20948 motion tracking sensor -- with DMP support

None of the ICM-20948 Arduino libraries contained support for the DMP (Digital Motion Processor), so I sorted through the Invensense library (worst code ever written) and wrote a layer on top of that to simplify everything.

The Invensense code loads the image file **icm20948_img.dmp3a.h** into the FPGA located on the ICM-20948. This FPGA is what does the onboard sensor fusion calculations to determine the quaternions.

Right now this library only has SPI support. I still need to clean up the code a bit more, but this should be a good start. Tested on a Teensy 4.1

Video demo:
https://streamable.com/ivmgfz

Application for the 3D animation: https://github.com/ZaneL/quaternion_sensor_3d_nodejs

Use the included example named **QuaternionAnimation** with that 3D app.

![alt text](https://i.ibb.co/VVMfQk9/image.png)
![alt text](https://i.ibb.co/SmM00g1/image.png)

The first example **ReadSensorValues** reads the various outputs on the ICM-20948 and writes their values to the USB serial port. The output frequency of each sensor is set to its respective minimum. For the quaternion output that is 50Hz, for the other three it is 1Hz.

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

The second example **QuaternionAnimation** enables only the quaternion output and sets it to the maximum possible output frequency. This output is written to the USB serial port and I've written a simple nodejs application that uses this data to plot the rotation of a 3D object in real time. That application can be found here: https://github.com/ZaneL/quaternion_sensor_3d_nodejs

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
