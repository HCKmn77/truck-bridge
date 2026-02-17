#ifndef SENSOR_H
#define SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

void read_gyro(float &x, float &y, float &z);

void sensor_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_H
