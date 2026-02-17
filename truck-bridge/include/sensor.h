#ifndef SENSOR_H
#define SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

bool imu_init(void);

// Read gyroscope data (degrees/second)
void read_gyro(float &x, float &y, float &z);

// Read accelerometer data (m/s²)
void read_accel(float &x, float &y, float &z);

void sensor_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_H
