#ifndef __TEENSY_ICM_20948_H__
#define __TEENSY_ICM_20948_H__

/*************************************************************************
  Defines
*************************************************************************/

typedef struct {
  int cs_pin;
  int spi_speed;
  int mode;
  bool enable_gyroscope;
  bool enable_accelerometer;
  bool enable_magnetometer;
  bool enable_quaternion;
  int gyroscope_frequency;
  int accelerometer_frequency;
  int magnetometer_frequency;
  int quaternion_frequency;

} TeensyICM20948Settings;

/*************************************************************************
  Class
*************************************************************************/

class TeensyICM20948
{
  public:

    TeensyICM20948();

    void init(TeensyICM20948Settings settings);
    void task();

    bool gyroDataIsReady();
    bool accelDataIsReady();
    bool magDataIsReady();
    bool quatDataIsReady();

    void readGyroData(float *x, float *y, float *z);
    void readAccelData(float *x, float *y, float *z);
    void readMagData(float *x, float *y, float *z);
    void readQuatData(float *w, float *x, float *y, float *z);
};


#endif // __TEENSY_ICM_20948_H__