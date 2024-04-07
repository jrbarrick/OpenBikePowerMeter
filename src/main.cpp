
#include <Wire.h>
#include <bluefruit.h>
#include <SPI.h>
#include <SD.h>
#include <float.h>

#include "ble_svcs.h"
#include "gyro_accl.h"
#include "load_cell.h"

//#define DEBUG
//#define DO_RAW_MEASURE_1S_LOOP
//#define COLLECT_REV_DATA
#define ENABLE_SD_LOGS
#define COLLECT_REV_STATS

#define FAKE_BAT_SOC 70

// define pins
#define VBATPIN A7
#define LED_PIN A6
#define SD_CS_PIN SS

// define constants
#define NO_OP_TOUT 10
#define NO_OP_SYSTEM_OFF_TOUT 60

#define MAX_UPDATE_INTRVL 2500
#define MIN_UPDATE_CADENCE 5

//#define APPLY_EXP_DECAY_ON_DPS_FCTR 0.9
//#define APPLY_EXP_DECAY_ON_FORCE_FCTR 0.9
//#define APPLY_EXP_DECAY_ON_POWER_FCTR 0.85

#define PEDALING_DET_SMPLS_THRSHLD  20

#define ENABLE_EXTREME_ANGLE_DETECT

#define ENABLE_DEAD_SPOT_DETECT
#define CONT_FORCE_DET_SMPL_COUNT 10
#define BUTTOM_DEAD_SPOT_MIN_FORCE_THRSHLD 3
#define TOP_DEAD_SPOT_MIN_FORCE_THRSHLD -1


#define USE_ANGL_INCR_REV_DETECT_WITH_FORCE_TRSHLD_BDS_INIT_SYNC

const float DPS_MIN = MIN_UPDATE_CADENCE * 6; //1000.f * (360.f / MAX_UPDATE_INTRVL);
const float DPS_CRANK_CIRC_VEL_MULTI =  PI * CRANK_RADIUS / 180;

uint16_t totalCrankRevs = 0;

bool idleConnected = false;

TaskHandle_t loopTaskHandle = NULL;
SemaphoreHandle_t loopTaskCmdDoneSemaphore = NULL;

enum LOOP_TASK_CMD {
  CALIBRATE_OFFSETS = 1,
  UPDATE_POWER = 2
};
float zForceCalibDiffL = 0, zForceCalibDiffR = 0;

#ifdef COLLECT_REV_STATS
struct {
  float avgAngVel = 0;
  float avgPowerPosL = 0;
  float avgPowerPosR = 0;
  float avgPowerNegL = 0;
  float avgPowerNegR = 0;
  float avgRawValueL = 0;
  float avgRawValueR = 0;
  float circVel = 0;
} lastRevStats;
#endif

struct {
  long  time = 0;
  float powerL = 0;
  float powerR = 0;
  float teL = 0;
  float teR = 0;
  float psL = 0;
  float psR = 0;
  float avgAngVel = 0;
  int16_t numOfSmpls = 0;
} lastRev;

#ifdef ENABLE_SD_LOGS
#define IDX_FILE "STARTCTR"
unsigned int START_IDX = 0;
String RIDE_PATH_PFX;
bool sdCardDetected = false;
File contRideLogFile;
#ifdef COLLECT_REV_DATA
  File contRevsLogFile;
  uint32_t totalRevInstForceValCtrL = 0;
#endif
inline bool initSD() {
  // Setup SD logger, if card is present
    if (SD.begin(SD_CS_PIN)) {
      START_IDX = 0;
       while (true) {
        RIDE_PATH_PFX = "ride" + String(START_IDX);
        if (!SD.exists(RIDE_PATH_PFX))
          break;
        else
          START_IDX++;
      };
      RIDE_PATH_PFX = "/" + RIDE_PATH_PFX + "/";
      sdCardDetected = true;
    }
#ifdef DEBUG
    else Serial.println(F("SD card not present!"));
#endif
  return sdCardDetected;
}
inline void syncSD() {
  if (sdCardDetected) {
    contRideLogFile.flush();
#ifdef COLLECT_REV_DATA
    contRevsLogFile.flush();
#endif
  }
}
inline void disableSD() {
  if (sdCardDetected) {
    SD.end();
  }
  pinMode(SS, INPUT);
  pinMode(MISO, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
}
inline void writeRideLogEntryToSD()  {
  if (sdCardDetected) {
    static char msg[1024];
    static bool initLog = true;
    if (initLog) {
      sprintf(msg, "Time; Cad; Power; Pedal; TEL; PSL; TER; PSR; Smpls");
      SD.mkdir(RIDE_PATH_PFX);
      contRideLogFile = SD.open(RIDE_PATH_PFX + "ride" + String(START_IDX) +".csv", FILE_WRITE);
      contRideLogFile.println(msg);
      initLog = false;
    }
    sprintf(msg, "%ld; %.3f; %.1f; %d; %.3f; %.3f; %.3f; %.3f; %d", lastRev.time, lastRev.avgAngVel / 6, lastRev.powerL + lastRev.powerR, lastRev.avgAngVel > DPS_MIN, lastRev.teL, lastRev.psL, lastRev.teR, lastRev.psR, lastRev.numOfSmpls);
    #ifdef DEBUG
    Serial.println(msg);
    #endif
    contRideLogFile.println(msg);
    contRideLogFile.flush();
  }
}
#ifdef COLLECT_REV_DATA
inline void initRevStatsOnSD() {
  static bool initLog = true;
  if (initLog) {
    SD.mkdir(RIDE_PATH_PFX);
    contRevsLogFile = SD.open(RIDE_PATH_PFX + "revs" + String(START_IDX) +".csv", FILE_WRITE);
    contRevsLogFile.println("smpl; instForceL; measTicks; pollWaitTicks");
    initLog = false;
  }
}
#endif

#ifdef DO_RAW_MEASURE_1S_LOOP
void storeAvgMeasurementsToSD() {
  float avgValueL, avgValueR;
  int minValueL, minValueR;
  int maxValueL, maxValueR;

  static char msg[1024];
#ifdef ENABLE_SD_LOGS
  static File contRawLogFile;
  if (sdCardDetected) {
    static String rawLogFile = "raw" + String(START_IDX) +".csv";
    static bool isNewLog = !SD.exists(rawLogFile);
    contRawLogFile = SD.open(rawLogFile, FILE_WRITE);
    if (isNewLog) {
      isNewLog = false;
      sprintf(msg, "Time; avgValueL; minValueL; maxValueL; dpsB; dpsE");
      contRawLogFile.println(msg);
      contRawLogFile.flush();
    }
  }
#endif

  float dpsBegin = gaGetAngularVelocity();
  if(!lcMeasureAndAvgWithMinMax(avgValueL, avgValueR, minValueL, minValueR, maxValueL, maxValueR, 3 * LOAD_CELL_EXP_SPS_RATE)) {
    Serial.println(F("ADC not available."));
    delay(1000);
  }
  float dpsEnd = gaGetAngularVelocity();
  sprintf(msg, "%ld; %.3f; %d; %d; %.3f; %.3f", millis(), avgValueL, minValueL, maxValueL, dpsBegin, dpsEnd) ;

#ifdef ENABLE_SD_LOGS
  if (sdCardDetected) {
    contRawLogFile.println(msg);
    contRawLogFile.flush();
  }
#endif // ENABLE_SD_LOGS
  Serial.println(msg);
}
#endif
#else
inline bool initSD() {return false;}
inline void syncSD() {}
inline void disableSD() {
  pinMode(LED_PIN, INPUT);
  pinMode(SS, INPUT);
  pinMode(MISO, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
}
inline void writeRideLogEntryToSD() {}
inline void writeRevStatsToSD() {}
#endif

void blinkLED(int count, int stayOnTime = 100, int stayOffTime = 100) {
  while(count) {
    digitalWrite(LED_PIN, 1);
    delay(stayOnTime);
    digitalWrite(LED_PIN, 0);
    count--;
    if (count)
      delay(stayOffTime);
  }
}

inline uint8_t getBatterySOC() {    
  #ifdef FAKE_BAT_SOC
    return FAKE_BAT_SOC;
  #endif

    float batV = analogRead(VBATPIN);
    batV = batV * 2 * 3.3 / 1024;

    if (batV > 4.1) {
      return 100;
    } else if (batV > 3.9) {
      return 90;
    } else if (batV > 3.7) {
      return 70;
    } else if (batV > 3.5) {
      return 40;
    } else if (batV > 3.3) {
      return 20;
    } else {
      return 5;
    }
}

inline void doInfrequentUpdateOperations() {
  // Other things (like battery) might be on a longer update schedule for power.
  static long lastInfrequentUpdate = millis();
  long timeNow = millis();
  // And check the battery, don't need to do it nearly this often though.
  // 1000 ms / sec * 60 sec / min * 5 = 5 minutes
  long timeDiff = timeNow - lastInfrequentUpdate;
  if (timeDiff == 0 || timeDiff > 1000 * 60 * 5) {
    blePublishBatt(getBatterySOC());
    lastInfrequentUpdate = timeNow;
  }
}

inline bool checkPedaling(float& angVel) {
  static uint32_t instPedlCtr = 0;
  static bool pedaling = false;

  if (angVel > DPS_MIN) {
    if(instPedlCtr < PEDALING_DET_SMPLS_THRSHLD) {
      instPedlCtr++;
    } else {
      pedaling = true;
    }
  } else if(instPedlCtr > 0) {
    instPedlCtr--;
  } else {
    pedaling = false;
  }
  return pedaling;
}

inline void applyExpDecay(float & currValue, float & avgValue, const float& avgWeight) {
  avgValue = currValue * avgWeight + (avgValue * (1 - avgWeight));
}

void isrCbLoadUpdate(float& instForceL, float& instForceR, int32_t& valueL, int32_t& valueR) {
  static uint32_t numOfPosPowerValsL = 0;
  static uint32_t numOfPosPowerValsR = 0;
  static uint32_t numOfNegPowerValsL = 0;
  static uint32_t numOfNegPowerValsR = 0;
  static uint32_t numOfSmpls = 0;

  static uint32_t maxPowerLIdx = 0;
  static uint32_t maxPowerRIdx = 0;
  static uint32_t minPowerRIdx = 0;
  static uint32_t minPowerLIdx = 0;

  static float maxPowerL = FLT_MIN;
  static float maxPowerR = FLT_MIN;
  static float minPowerR = FLT_MAX;
  static float minPowerL = FLT_MAX;
  
  static bool bdsPreCondReached = false;
  static bool tdsPreCondReached = false;
  static bool bdsReached = false;
  static bool tdsReached = false;
  
  static uint32_t tdsLIdx = 0;
  static uint32_t bdsLIdx = 0;

  static double sumAngVel = 0.f;
  static double sumPowerPosL = 0.f;
  static double sumPowerPosR = 0.f;
  static double sumPowerNegL = 0.f;
  static double sumPowerNegR = 0.f;

  #ifdef COLLECT_REV_STATS
  static double sumValueL = 0.f;
  static double sumValueR = 0.f;
  #endif

  static float angVel = 0;
  static float forceL = 0;
  static float forceR = 0;

  #ifdef USE_ANGL_INCR_REV_DETECT_WITH_FORCE_TRSHLD_BDS_INIT_SYNC
  static float revAngl = 0;
  static bool  syncPedaling = true;
  static float prevForceL = 0;
  #endif

  static long lastUpdate = millis();

  float instDPS = gaGetAngularVelocity();
  instDPS = (instDPS < 90 || instDPS > 1440) ? 0: abs(instDPS); // filter ang. vel. to allow only pos. cad. >15rpm & <240rpm
  #ifdef APPLY_EXP_DECAY_ON_DPS_FCTR
  applyExpDecay(instDPS, angVel, APPLY_EXP_DECAY_ON_DPS_FCTR);
  #else
  angVel = instDPS;
  #endif

  long smplTime = millis();
  
  #ifdef APPLY_EXP_DECAY_ON_FORCE_FCTR
  applyExpDecay(instForceL, forceL, APPLY_EXP_DECAY_ON_FORCE_FCTR);
  applyExpDecay(instForceR, forceR, APPLY_EXP_DECAY_ON_FORCE_FCTR);
  #else
  forceL = instForceL;
  forceR = instForceR;
  #endif

  float instCircVel = angVel * DPS_CRANK_CIRC_VEL_MULTI;

  sumAngVel += angVel;
  #ifdef COLLECT_REV_STATS
  sumValueL += valueL;
  sumValueR += valueR;
  #endif

  float instPwrL = forceL * instCircVel;
  float instPwrR = forceR * instCircVel;

  if (instPwrL > 0) {
    sumPowerPosL += instPwrL;
    numOfPosPowerValsL++;
  }
  if (instPwrR > 0) {
    sumPowerPosR += instPwrR;
    numOfPosPowerValsR++;
  }
  if (instPwrL < 0) {
    sumPowerNegL += instPwrL;
    numOfNegPowerValsL++;
  }
  if (instPwrR < 0) {
    sumPowerNegR += instPwrR;
    numOfNegPowerValsR++;
  }
  if (instPwrL < minPowerL) minPowerL = instPwrL;
  if (instPwrL > maxPowerL) maxPowerL = instPwrL;
  if (instPwrR < minPowerR) minPowerR = instPwrR;
  if (instPwrR > maxPowerR) maxPowerR = instPwrR;

  #ifdef ENABLE_EXTREME_ANGLE_DETECT
  if (instPwrL < minPowerL) minPowerLIdx = numOfSmpls;
  if (instPwrL > maxPowerL) maxPowerLIdx = numOfSmpls;
  if (instPwrR < minPowerR) minPowerRIdx = numOfSmpls;
  if (instPwrR > maxPowerR) maxPowerRIdx = numOfSmpls;
  #endif
  
  numOfSmpls++;

  #ifdef ENABLE_DEAD_SPOT_DETECT
  //bool bdsReached  = (prevForceL > BUTTOM_DEAD_SPOT_MIN_FORCE_THRSHLD) && (prevForceL > forceL) && (forceL < BUTTOM_DEAD_SPOT_MIN_FORCE_THRSHLD);
  if (!bdsPreCondReached) {
    static int32_t contDecrForceLCtr = 0;
    if (forceL < prevForceL) {
      contDecrForceLCtr++;
    } else {
      contDecrForceLCtr = 0;
    }
    if ((contDecrForceLCtr > CONT_FORCE_DET_SMPL_COUNT) && (forceL < BUTTOM_DEAD_SPOT_MIN_FORCE_THRSHLD)) {
      contDecrForceLCtr = 0;
      bdsPreCondReached = true;
    }
  }
  if (!tdsPreCondReached) {
    static int32_t contIncrForceLCtr = 0;
    if (forceL > prevForceL) {
      contIncrForceLCtr++;
    } else {
      contIncrForceLCtr = 0;
    }
    if ((contIncrForceLCtr > CONT_FORCE_DET_SMPL_COUNT) && (forceL > TOP_DEAD_SPOT_MIN_FORCE_THRSHLD)) {
      contIncrForceLCtr = 0;
      tdsPreCondReached = true;
    }
  }
  if (!tdsReached && tdsPreCondReached && (prevForceL < 0) && (prevForceL < forceL) && (forceL > 0)) {
    tdsReached = true;
    tdsLIdx = numOfSmpls;
  }
  if (!bdsReached && bdsPreCondReached && (prevForceL > 0) && (prevForceL > forceL) && (forceL < 0)) {
    bdsReached = true;
    bdsLIdx = numOfSmpls;
  }
  prevForceL = forceL;
  #endif

  bool updatePower = false;
  #ifdef USE_TIME_BASED_INST_ANGL_REV_DETECT
  if (checkPedaling(angVel) && (smplTime - lastUpdate) < (360.f / angVel) * 1000.f)) {
    totalCrankRevs += 1;
    updatePower = true;
  }
  #elif defined(ENABLE_DEAD_SPOT_DETECT) && defined(USE_ANGL_INCR_REV_DETECT_WITH_FORCE_TRSHLD_BDS_INIT_SYNC)
  if (checkPedaling(angVel)) {
    revAngl += angVel / LOAD_CELL_EXP_SPS_RATE;
    if (syncPedaling) {
      // synchronize revolution if left crank arm is in down stroke and buttom dead-spot is reached
      if (bdsReached) {
        syncPedaling = false;
        updatePower = true;
      }
    } else if (revAngl >= 360) {
      revAngl -= 360;
      updatePower = true;
      totalCrankRevs += 1;
    }
  } else {
    revAngl = 0;
    syncPedaling = true;
  }
  #endif



  if (updatePower) {
    float avgAngVel = sumAngVel / numOfSmpls;
    float avgPowerL = (sumPowerPosL + sumPowerNegL) / numOfSmpls;
    float avgPowerR = (sumPowerPosR + sumPowerNegR) / numOfSmpls;

    #ifdef APPLY_EXP_DECAY_ON_POWER_FCTR
    applyExpDecay(avgPowerL, lastRev.powerL, APPLY_EXP_DECAY_ON_POWER_FCTR);
    applyExpDecay(avgPowerR, lastRev.powerR, APPLY_EXP_DECAY_ON_POWER_FCTR);
    #else
    lastRev.powerL = avgPowerL;
    lastRev.powerR = avgPowerR;
    #endif

    float avgPowerPosL = (numOfPosPowerValsL > 1) ? sumPowerPosL / numOfPosPowerValsL : sumPowerPosL;
    float avgPowerPosR = (numOfPosPowerValsR > 1) ? sumPowerPosR / numOfPosPowerValsR : sumPowerPosR;
    float avgPowerNegL = (numOfNegPowerValsL > 1) ? sumPowerNegL / numOfNegPowerValsL : sumPowerNegL;
    float avgPowerNegR = (numOfNegPowerValsR > 1) ? sumPowerNegR / numOfNegPowerValsR : sumPowerNegR;

    lastRev.teL = (avgPowerPosL + avgPowerNegL) / avgPowerPosL;
    lastRev.teR = (avgPowerPosR + avgPowerNegR) / avgPowerPosR;
    lastRev.psL = avgPowerL / maxPowerL;
    lastRev.psR = avgPowerR / maxPowerR;

    lastRev.avgAngVel  = avgAngVel;
    lastRev.numOfSmpls = numOfSmpls;

    #ifdef ENABLE_DEAD_SPOT_DETECT
    if (bdsReached) {
    }
    if (tdsReached) {
    }
    #endif

    #ifdef COLLECT_REV_STATS
    lastRevStats.circVel   = avgAngVel * DPS_CRANK_CIRC_VEL_MULTI;
    lastRevStats.avgAngVel = avgAngVel;
    lastRevStats.avgPowerPosL = avgPowerPosL;
    lastRevStats.avgPowerPosR = avgPowerPosR;
    lastRevStats.avgPowerNegL = avgPowerNegL;
    lastRevStats.avgPowerNegR = avgPowerNegR;
    lastRevStats.avgRawValueL = sumValueL / numOfSmpls;
    lastRevStats.avgRawValueR = sumValueR / numOfSmpls;
    #endif
    
  } else if((smplTime - lastUpdate) >= MAX_UPDATE_INTRVL) {
    memset(&lastRev, 0, sizeof(lastRev));
    #ifdef COLLECT_REV_STATS
    memset(&lastRevStats, 0, sizeof(lastRevStats));
    #endif
  } else {
    return;
  }

  lastRev.time = smplTime;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(loopTaskHandle, LOOP_TASK_CMD::UPDATE_POWER, eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);
  
  lastUpdate = smplTime;

  maxPowerL = FLT_MIN;
  maxPowerR = FLT_MIN;
  minPowerR = FLT_MAX;
  minPowerL = FLT_MAX;
  numOfPosPowerValsL = 0;
  numOfPosPowerValsR = 0;
  numOfNegPowerValsL = 0;
  numOfNegPowerValsR = 0;
  numOfSmpls = 0;
  sumAngVel = 0;
  sumPowerPosL = 0;
  sumPowerPosR = 0;
  sumPowerNegL = 0;
  sumPowerNegR = 0;
  bdsPreCondReached = false;
  tdsPreCondReached = false;
  bdsReached = false;
  tdsReached = false;
  #ifdef COLLECT_REV_STATS
  sumValueL = 0.f;
  sumValueR = 0.f;
  #endif
}

bool syncCmdExec(uint32_t cmd, uint32_t waitTime)
{
  if(xSemaphoreTake(loopTaskCmdDoneSemaphore, ms2tick(waitTime))) {
    xTaskNotify(loopTaskHandle, cmd, eSetValueWithoutOverwrite);
    if(xSemaphoreTake(loopTaskCmdDoneSemaphore, ms2tick(waitTime))) {
      xSemaphoreGive(loopTaskCmdDoneSemaphore);
      return true;
    }
  }
  return false;
}

int16_t doOffsetCalibrations(bool& done) {
  if (!idleConnected) {
    done = false;
    return 0;
  }
  done = syncCmdExec(CALIBRATE_OFFSETS, 3000);
  #ifdef ENABLE_RIGHT_LOAD_CELL
  float diff = abs(zForceCalibDiffL) + abs(zForceCalibDiffR);
  #else
  float diff = zForceCalibDiffL;
  #endif
  return diff;// * 1000;
}

inline void systemOff() {
  disableSD();
  //sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  //sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
  //__WFE();
  //__WFI();
  //sd_app_evt_wait() ; //SOFT DEVICE “SYSTEM ON” POWER SAVE
  sd_power_system_off();
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  // blink once to confirm startup
  blinkLED(1);
  
  lcSetup();
  lcSetScaleFactor(LOAD_FORCE_SCALE_FCT_L, LOAD_FORCE_SCALE_FCT_R);

  lcSoftPowerUp();
  lcDoOffsetCalib(LOAD_CELL_EXP_SPS_RATE);
  lcSoftPowerDown();

  Wire.begin();
  Serial.begin(115200);

  gaSetup();
  bleSetup();

  if(!initSD()) {
    disableSD();
  }


  loopTaskHandle = xTaskGetCurrentTaskHandle();
  loopTaskCmdDoneSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(loopTaskCmdDoneSemaphore);

  vTaskPrioritySet(NULL, TASK_PRIO_HIGHEST);

  #ifdef COLLECT_REV_STATS
  memset(&lastRevStats, 0, sizeof(lastRevStats));
  #endif

  bleSetOffsetCompensationCb(doOffsetCalibrations);

  // blink twice to confirm setup complete
  blinkLED(2);
}

void loop() {
  static uint32_t idleCtr = 0;
  TickType_t loopWakeTime = xTaskGetTickCount();

  #ifdef DO_RAW_MEASURE_1S_LOOP
  storeAvgMeasurementsToSD();
  return;
  #endif

  if (Bluefruit.connected()) {
  static bool init = true;
    if (idleCtr >= NO_OP_TOUT || init) {
      gaDisableCycledSleep();
      lcSoftPowerUp();
    }
    if (idleCtr > 0 || init) {      
      lcStartContValueUpdate(isrCbLoadUpdate);
      idleCtr = 0;
      init = false;
    }

    uint32_t notifiedCommand = 0;
    xTaskNotifyWait(0xffffffff, 0, &notifiedCommand, ms2tick(5000));
    idleConnected = (lastRev.avgAngVel == 0);
    
    if (notifiedCommand) {
      switch (notifiedCommand)
      {
      case LOOP_TASK_CMD::CALIBRATE_OFFSETS:
        if(!idleConnected) {
          break;
        }
        lcStopContValueUpdate();
        float zForcePreCalibL, zForcePreCalibR;
        lcGetValue(zForcePreCalibL, zForcePreCalibR);
        lcDoOffsetCalib(LOAD_CELL_EXP_SPS_RATE);
        lcGetValue(zForceCalibDiffL, zForceCalibDiffR);
        zForceCalibDiffL -= zForcePreCalibL;
        zForceCalibDiffR -= zForcePreCalibR;
        gaGyroZeroCalibration();
        lcStartContValueUpdate(isrCbLoadUpdate);
        break;
      case LOOP_TASK_CMD::UPDATE_POWER:
        blePublishPower(lastRev.powerL, lastRev.powerR, totalCrankRevs, lastRev.time);
        blePublishTePs(lastRev.teL, lastRev.psL);
      break;
      default:
        break;
      }
      xSemaphoreGive(loopTaskCmdDoneSemaphore);
    }
 
    doInfrequentUpdateOperations();

    #ifdef COLLECT_REV_STATS
    #ifdef ENABLE_BLE_LOG
    float scaleFctL, scaleFctR;
    lcGetScaleFactor(scaleFctL, scaleFctR);
    float avgForceL = lastRevStats.avgRawValueL * scaleFctL;
    float avgForceR = lastRevStats.avgRawValueR * scaleFctR;
    float avgForceLS = avgForceL > avgForceR ? avgForceL : avgForceR;

    blePublishLog("P%.0f F%.1f C%.0f", lastRev.powerL + lastRev.powerR, avgForceLS, lastRev.avgAngVel / 6);
    #ifdef MANUAL_CALIB
    float avgRawValueLS = lastRevStats.avgRawValueL > lastRevStats.avgRawValueR ? lastRevStats.avgRawValueL : lastRevStats.avgRawValueR;
    static float rlMax = 0;
    if (!idleConnected) {
      rlMax = 0;
    } else {
      blePublishLog("F%.2f RL%.3f", avgForceLS, avgRawValueLS);
      if (avgRawValueLS > rlMax) {
        rlMax = avgRawValueLS;
      }
      delay(1000);
      blePublishLog("RLmax%.3f", rlMax);
    }
    #endif
    #endif
    #endif

    #ifdef ENABLE_SD_LOGS
    writeRideLogEntryToSD();
    #endif
  } else {
    blinkLED(1, 100);
    switch (idleCtr) {
    case 0:
      lcStopContValueUpdate();
    break;
    case NO_OP_TOUT:
      syncSD();
      lcSoftPowerDown();
      gaEnableCycledSleep();
    break;
    case NO_OP_SYSTEM_OFF_TOUT:
      systemOff();
      break;
    default:
      break;
    }
    idleCtr++;
    vTaskDelayUntil(&loopWakeTime, 1000);
  }
}
