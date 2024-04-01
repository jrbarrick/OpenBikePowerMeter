#ifndef _GYRO_ACCL_H_
#define _GYRO_ACCL_H_

#define CRANK_RADIUS 0.170

void  gaSetup();
float gaGetAngularVelocity();
float gaGetTemperature();
void  gaEnableCycledSleep();
void  gaDisableCycledSleep();
void  gaGyroZeroCalibration();

#endif //_GYRO_ACCL_H_
