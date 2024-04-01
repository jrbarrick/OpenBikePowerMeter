/**
 * MPU6050/gyroscope specific code.
 */

#include "gyro_accl.h"
#include "MPU6050.h"

#define GYRO_INT_PIN A2

#define GYRO_CALIBRATION_SAMPLES 100

#define ROTATION_DIR_Z -1

#define GYRO_OFFSET 0

//#define USE_ZMOT_DETECT

#define GYRO_RANGE MPU6050_GYRO_FS_1000

// At +/- 250 degrees/s, the LSB/deg/s is 131. Per the mpu6050 spec.
/* FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
*/
float gyroSENS[4] = {131, 65.5, 32.8, 16.4};

MPU6050 mpu;

void motionDetectChange() {
  mpu.getIntDataReadyStatus();
}

void gaEnableCycledSleep() {
  #ifdef USE_ZMOT_DETECT
  mpu.setIntZeroMotionEnabled(true);
  #else
  mpu.setIntMotionEnabled(true);
  #endif
  mpu.getIntDataReadyStatus();
  mpu.setWakeCycleEnabled(true);
}

void gaDisableCycledSleep() {
  #ifdef USE_ZMOT_DETECT
  mpu.setIntZeroMotionEnabled(false);
  #else
  mpu.setIntMotionEnabled(false);
  #endif
  mpu.setWakeCycleEnabled(false);
}

/**
 *  Initialize MPU and setup all used features
 */
void gaSetup() {
  pinMode(GYRO_INT_PIN, INPUT_SENSE_HIGH);
  attachInterrupt(digitalPinToInterrupt(GYRO_INT_PIN), motionDetectChange, RISING);
  
  if (mpu.getWakeCycleEnabled()) {
    mpu.getIntDataReadyStatus();
    //gyroDisablePowerSave();
    return;
  }

  // initialize MPU
  mpu.initialize();

  mpu.setFullScaleGyroRange(GYRO_RANGE);
  mpu.setAccelerometerPowerOnDelay(3);
  mpu.setDHPFMode(1); //needs more testing
  
  // disable unused stuff
  mpu.setDMPEnabled(false);

  mpu.setStandbyXGyroEnabled(true);
  mpu.setStandbyYGyroEnabled(true);
  //mpu.setStandbyZGyroEnabled(false);

  //mpu.setStandbyXAccelEnabled(true);
  //mpu.setStandbyYAccelEnabled(true);
  //mpu.setStandbyZAccelEnabled(false);

#ifdef USE_ZMOT_DETECT
  // Set zero motion detection
  //mpu.setIntZeroMotionEnabled(true);
  mpu.setZeroMotionDetectionThreshold(4);
  // 1 LSB = 64ms. So 30s = 
  mpu.setZeroMotionDetectionDuration(80);
  mpu.setInterruptLatchClear(true);
  //mpu.setInterruptLatch(false);
#else
  //mpu.setIntMotionEnabled(true);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(2);
  mpu.setInterruptLatchClear(true);
  //mpu.setInterruptLatch(false);
#endif

  mpu.setWakeFrequency(1);
  mpu.setWakeCycleEnabled(true);

  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // TODO: apply proper zero calibration for the rotation axis. for now apply a fixed constant...
  mpu.setZGyroOffset(GYRO_OFFSET);
}

void gaGyroZeroCalibration() {
// TODO make a calibration mode. For now, test manually.
// 5 tests sitting at the kitchen table, offsets were   -39 -39 -39 -38 -38

  // Calibrate the gyro
  mpu.setZGyroOffset(0);
  double sumZ = 0;
  int16_t maxSample = -32768;
  int16_t minSample = 32767;
  // Read n-samples
  for (uint8_t i = 0; i < GYRO_CALIBRATION_SAMPLES; ++i) {
    delay(1);
    int16_t reading = mpu.getRotationZ();
    if (reading > maxSample) maxSample = reading;
    if (reading < minSample) minSample = reading;
    sumZ += reading;
  }

  // Throw out the two outliers
  sumZ -= minSample;
  sumZ -= maxSample;

  // Two fewer than the calibration samples because we took out two outliers.
  float deltaZ = sumZ / (GYRO_CALIBRATION_SAMPLES - 2);
  deltaZ = -1 * deltaZ;
#ifdef DEBUG
  Serial.printf("Discounting max (%d) and min (%d) samples.\n", maxSample, minSample);
  Serial.printf("Gyro calculated offset: %f\n", deltaZ); 
#endif // DEBUG
  mpu.setZGyroOffset(deltaZ);
}

float gaGetAngularVelocity() {
  return mpu.getRotationZ() / gyroSENS[GYRO_RANGE] * ROTATION_DIR_Z;
}

float gaGetTemperature() {
  return mpu.getTemperature() / 340.f + 36.53;
}

#if 0
/**
 * This doesn't do anything but echo applied setting on the MPU.
 */
void dumpSettings() {
  Serial.println();
  Serial.printf(" * Gyroscope Sleep Mode: %d\n", mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  Serial.printf(" * Gyroscope offset:     %d\n", mpu.getZGyroOffset());
}

/**
 * Gets a normalized averaged rotational velocity calculation. The MPU6050 library supports a
 * normalized gyroscope reading, which trims off outliers and scales the values to deg/s.
 *
 * An exponential average is applied to further smooth data, with weight of WEIGHT. I don't
 * love this, becaues it means no value is every entirely discarded, but exponential decay
 * probably makes it effectively the same. Maybe something to revisit.
 *
 * Returns a value for foot speed, in degrees/second.
 */
void getNormalAvgVelocity(float & avgValue, const double& avgWeight) {
  // At +/- 250 degrees/s, the LSB/deg/s is 131. Per the mpu6050 spec.
  /* FS_SEL | Full Scale Range   | LSB Sensitivity
   * -------+--------------------+----------------
   * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
   * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
   * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
   * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
  */
  const static float SENSITIVITY = 32.8;

  // Request new data from the MPU. The orientation obviously dictates
  // which x/y/z value we're interested in, but right now Z.
  // Use the absolute value. Cause who knows if the chip is just backwards.
  float rotz = abs(mpu.getRotationZ() / SENSITIVITY);
  if (rotz < 90) {
    // Magic number here, but less than 90 dps is less than 1 crank rotation 
    // in 4 seconds (15 RPM), just assume it's noise from the road bumps.
    rotz = 0.f;
  }
  // Return a rolling average, including the last reading.
  // e.g. if weight is 0.90, it's 10% what it used to be, 90% this new reading.
  avgValue = (rotz * avgWeight) + (avgValue * (1 - avgWeight));
}

/**
 * Provide the average rate, in degrees/second.
 *
 * Returns the circular velocity of the rider's foot. Takes in the crank's averaged rotational
 * velocity, converts it to radians, multiplies by the crank radius, and returns the converted
 * value.
 *
 * Value returned is in meters/second
 */
float getCircularVelocity(const float & dps) {
  // 2 * PI * radians = 360 degrees  -- by definition
  // dps degrees/second * (PI / 180) rad/degree = rad/second
  // (rad/sec) / (rad/circumference) = (rad/sec) / 2 * PI = ratio of a circumference traveled, still per second
  // 2 * PI * CRANK_RADIUS = circumference  -- by definition, that's a circle
  // ratio of circumference traveled * circumference = meters/second

  // It all comes out to:
  // m/s = ((dps * (PI / 180)) / (2 * PI)) * (2 * PI * CRANK_RADIUS);
  // And simplifies to:
  return (dps * PI * CRANK_RADIUS) / 180;
}

/**
 *  Provide angular velocity, degrees/sec.
 *
 *  Returns a new cadence measurement.
 *
 *  Note this isn't necessary for power measurement, but it's a gimme addon
 *  given what we already have and useful for the athlete.
 *
 *  Returns an int16 of cadence, rotations/minute.
 */
inline int16_t getCadence(float dps) {
  // Cadence is the normalized angular velocity, times 60/360, which
  // converts from deg/s to rotations/min. x * (60/360) = x / 6.
  return dps / 6;
}

/**
 *  Determine current angle of the crank arm. Based on the acceleration
 *  for gravity.
 */
inline int16_t getAngle() {
  // Sensitivity for 2g is 48
  static const int16_t SENS = 48;

  // TODO not certain how to do this yet. If we calibrate on the fly to
  // get known values, we have to worry about the orientation of the cranks
  // when that's done. We could calibrate as a 1-time thing but that's less
  // preferable because.. what if it drifts? If we don't calibrate, that
  // could still be ok, because what we really want is to know the "peaks",
  // min and max. Those are when the cranks are perpendicular to the ground.
  // And the mins are straight up and down. Downside there is that the max
  // values will change with the acceleration of cadence, so we'd have to
  // almost continuously figure out what they are?

  // For now, just return the raw X acceleration.
  return mpu.getAccelerationX() / SENS;
}
#endif