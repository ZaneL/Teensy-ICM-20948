# ICM-20948 library for Arduino with DMP support

None of the ICM-20948 Arduino libraries contained support for the DMP (Digital Motion Processor), so I sorted through the Invensense library (worst code ever written) and wrote a layer on top of that to simplify everything.

The Invensense code loads the image file **icm20948_img.dmp3a.h** into the FPGA located on the ICM-20948. This FPGA is what does the onboard sensor fusion calculations to determine the quaternions.

Right now this library only has SPI support. I can add I2C support if there is any interest. It should also be easy enough to add step-counter and gesture functionality if anyone is interested...

The example code shows how to use the library.

    #include <Teensy-ICM-20948.h>
    
    TeensyICM20948 icm20948;
    
    void setup()
    {
      Serial.begin(115200);
      delay(2000);
    
      // Use pin number 10 for SPI chip select
      icm20948.init(10);
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

