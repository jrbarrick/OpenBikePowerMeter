
#include <Wire.h>
#include <bluefruit.h>
#include <SPI.h>
#include <SD.h>
#include <float.h>

#include "ble_svcs.h"
#include "gyro_accl.h"
#include "load_cell.h"
#include "cfg.h"


//#define DEBUG
//#define DO_RAW_MEASURE_1S_LOOP
//#define COLLECT_REV_DATA
#define ENABLE_SD_LOGS
#define COLLECT_REV_STATS

//#define ENABLE_EXTREME_ANGLE_DETECT

#define FAKE_BAT_SOC 70

// define pins
#define VBAT_PIN PIN_VBAT
#define LED_PIN  PIN_LED2
#define SD_CS_PIN SS

// define constants
#define NO_OP_TOUT 10
#define NO_OP_SYSTEM_OFF_TOUT 60

#define MAX_UPDATE_INTRVL 2500
#define MIN_UPDATE_CADENCE 5

#define DEFAULT_CRANK_RADIUS 170
#define DEFAULT_EXP_DECAY_ON_POWER_FCTR 0.9
#define DEFAULT_AVG_POWER_OVER_N_REVS 3

//#define APPLY_EXP_DECAY_ON_DPS_FCTR 0.9
//#define APPLY_EXP_DECAY_ON_FORCE_FCTR 0.9

#define ENABLE_DEAD_SPOT_DETECT
#define CONT_FORCE_DET_SMPL_COUNT 10
#define BUTTOM_DEAD_SPOT_MIN_FORCE_THRSHLD 3
#define TOP_DEAD_SPOT_MIN_FORCE_THRSHLD -1
#define PEDALING_DET_SMPLS_THRSHLD  20

#define USE_ANGL_INCR_REV_DETECT_WITH_FORCE_TRSHLD_BDS_INIT_SYNC

const float DPS_MIN = MIN_UPDATE_CADENCE * 6; //1000.f * (360.f / MAX_UPDATE_INTRVL);

float DPS_CRANK_CIRC_VEL_MULTI =  PI * DEFAULT_CRANK_RADIUS / 1000.f / 180.f;

uint16_t totalCrankRevs = 0;

bool idleConnected = false;

TaskHandle_t loopTaskHandle = NULL;
SemaphoreHandle_t loopTaskCmdDoneSemaphore = NULL;

enum CALIB_MODE {
  CALIB_NOT_ACTIVE = 0,
  CALIB_L = 1,
  CALIB_R = 2,
  CALIB_TEST = 3
};

enum LOOP_TASK_CMD {
  CMD_INVALID = 0,
  CALIBRATE_OFFSETS = 1,
  UPDATE_POWER = 2,
  CALIB_LC_L = 3,
  CALIB_LC_R = 4,
  CALIB_APPLY = 6,
  CALIB_VERIFY = 7,
  CALIB_PERSIST = 8
};
float zForceCalibDiffL = 0, zForceCalibDiffR = 0;

CfgCalib cfgClb = {
  .cfgVer = CFG_CLB_VER,
  .lcForceCalibFctL = 1666,
  .lcForceCalibFctR = 1666
};
CfgRuntime cfgRt = {
  .cfgVer = CFG_RT_VER
};

enum BLE_LOG_MODES {
  BLE_LOG_DISABLED = 0,
  BLE_LOG_POWER_HR
};
uint32_t bleLogMode = BLE_LOG_POWER_HR;

bool lcActiveL = false, lcActiveR = false;

float lcClbForce = 9.999f * 9.80928f;

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
  float avgAngVel = 0;
  int16_t numOfSmpls = 0;
} lastRevMI;

revUpdt_t lastRev;


void applyDefaultRuntimeCfg() {
  #ifdef DEFAULT_CRANK_RADIUS
  cfgRt.crankRadius = DEFAULT_CRANK_RADIUS;
  #else
  cfgRt.pwrAvgRevs = 1;
  #endif
  #ifdef DEFAULT_AVG_POWER_OVER_N_REVS
  cfgRt.pwrAvgRevs = DEFAULT_AVG_POWER_OVER_N_REVS;
  #else
  cfgRt.pwrAvgRevs = 1;
  #endif
  #ifdef DEFAULT_EXP_DECAY_ON_POWER_FCTR
  cfgRt.pwrExpDecFctr = DEFAULT_EXP_DECAY_ON_POWER_FCTR;
  #else
  cfgRt.pwrExpDecFctr = 1;
  #endif
  cfgRt.instPwrMeas = false;
  //TODO:...
}






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
    sprintf(msg, "%ld; %.3f; %.1f; %d; %.3f; %.3f; %.3f; %.3f; %d", lastRev.time, lastRevMI.avgAngVel / 6, lastRev.powerL + lastRev.powerR, lastRevMI.avgAngVel > DPS_MIN, lastRev.teL, lastRev.psL, lastRev.teR, lastRev.psR, lastRevMI.numOfSmpls);
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

    float batV = analogRead(VBAT_PIN);
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
    blePublishBatSOC(getBatterySOC());
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
  
  #ifdef ENABLE_EXTREME_ANGLE_DETECT
  static uint32_t maxPowerLIdx = 0;
  static uint32_t maxPowerRIdx = 0;
  static uint32_t minPowerRIdx = 0;
  static uint32_t minPowerLIdx = 0;
  #endif
  static float maxPowerL = FLT_MIN;
  static float maxPowerR = FLT_MIN;
  static float minPowerR = FLT_MAX;
  static float minPowerL = FLT_MAX;
  
  static bool bdsPreCondReached = false;
  static bool tdsPreCondReached = false;
  static bool bdsReached = false;
  static bool tdsReached = false;

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
  }
  if (!bdsReached && bdsPreCondReached && (prevForceL > 0) && (prevForceL > forceL) && (forceL < 0)) {
    bdsReached = true;
  }
  prevForceL = forceL;
  #endif



  numOfSmpls++;



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
        updatePower = cfgRt.instPwrMeas;
        revAngl = 0;
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


    float pwrL = lastRev.powerL;
    float pwrR = lastRev.powerR;
    if (cfgRt.pwrExpDecFctr < 1) {
      applyExpDecay(avgPowerL, pwrL, cfgRt.pwrExpDecFctr);
      applyExpDecay(avgPowerR, pwrR, cfgRt.pwrExpDecFctr);
    } else {
      pwrL = avgPowerL;
      pwrR = avgPowerR;
    }
    if (cfgRt.pwrAvgRevs > 1) {
      lastRev.powerL = (lastRev.powerL * (cfgRt.pwrAvgRevs - 1) + pwrL) / cfgRt.pwrAvgRevs;
      lastRev.powerR = (lastRev.powerR * (cfgRt.pwrAvgRevs - 1) + pwrR) / cfgRt.pwrAvgRevs;
    } else 
    {
      lastRev.powerL = pwrL;
      lastRev.powerR = pwrR;
    }


    float avgPowerPosL = (numOfPosPowerValsL > 1) ? sumPowerPosL / numOfPosPowerValsL : sumPowerPosL;
    float avgPowerPosR = (numOfPosPowerValsR > 1) ? sumPowerPosR / numOfPosPowerValsR : sumPowerPosR;
    float avgPowerNegL = (numOfNegPowerValsL > 1) ? sumPowerNegL / numOfNegPowerValsL : sumPowerNegL;
    float avgPowerNegR = (numOfNegPowerValsR > 1) ? sumPowerNegR / numOfNegPowerValsR : sumPowerNegR;

    lastRev.teL = (avgPowerPosL + avgPowerNegL) / avgPowerPosL;
    lastRev.teR = (avgPowerPosR + avgPowerNegR) / avgPowerPosR;
    lastRev.psL = avgPowerL / maxPowerL;
    lastRev.psR = avgPowerR / maxPowerR;

    #ifdef ENABLE_DEAD_SPOT_DETECT
    if (bdsReached) {
      //TODO:...
    }
    if (tdsReached) {
      //TODO:...
    }
    #endif

    #ifdef ENABLE_EXTREME_ANGLE_DETECT
    //TODO:...
    #endif

    lastRevMI.avgAngVel  = avgAngVel;
    lastRevMI.numOfSmpls = numOfSmpls;

    #ifdef COLLECT_REV_STATS
    lastRevStats.circVel   = avgAngVel * DPS_CRANK_CIRC_VEL_MULTI;
    lastRevStats.avgAngVel = avgAngVel;
    lastRevStats.avgPowerPosL = avgPowerPosL;
    lastRevStats.avgPowerPosR = avgPowerPosR;
    lastRevStats.avgPowerNegL = avgPowerNegL;
    lastRevStats.avgPowerNegR = avgPowerNegR;
    #endif
    
  } else if((smplTime - lastUpdate) >= MAX_UPDATE_INTRVL) {
    memset(&lastRev, 0, sizeof(lastRev));
    memset(&lastRevMI, 0, sizeof(lastRevMI));
    #ifdef COLLECT_REV_STATS
    memset(&lastRevStats, 0, sizeof(lastRevStats));
    #endif
  } else {
    return;
  }

  #ifdef COLLECT_REV_STATS
  lastRevStats.avgRawValueL = sumValueL / numOfSmpls;
  lastRevStats.avgRawValueR = sumValueR / numOfSmpls;
  #endif

  lastRev.time = smplTime;  
  lastRev.crankRev = totalCrankRevs;

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

void printActiveCfg() {
  Serial.println(F("Active Configuration:"));
  Serial.print(F(" LoadCellAvailableL: ")); Serial.println(lcActiveL);
  Serial.print(F(" LoadCellAvailableR: ")); Serial.println(lcActiveR);
  Serial.print(F(" ForceCalibFctL: ")); Serial.println(cfgClb.lcForceCalibFctL);
  Serial.print(F(" ForceCalibFctR: ")); Serial.println(cfgClb.lcForceCalibFctR);
  Serial.print(F(" PwrAvgRevs:     ")); Serial.println(cfgRt.pwrAvgRevs);
  Serial.print(F(" PwrExpDecFctr:  ")); Serial.println(cfgRt.pwrExpDecFctr);
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

int16_t doProcessCfmRequest(cfmReq_t req, uint8_t* respBuf, float arg1, float arg2, float arg3) {
  uint32_t cmd = CMD_INVALID;
  switch (req)
  {
  case REQ_CALIB_AUTO_L:
    lcClbForce = arg1;
    cmd = CALIB_LC_L;
    break;
  case REQ_CALIB_AUTO_R:
    lcClbForce = arg1;
    cmd = CALIB_LC_R;
    break;
  case REQ_CALIB_SET_L:
    cfgClb.lcForceCalibFctL = arg1;
    cmd = CALIB_PERSIST;
    break;
  case REQ_CALIB_SET_R:
    cfgClb.lcForceCalibFctR = arg1;
    cmd = CALIB_PERSIST;
    break;
  case REQ_CALIB_GET:
    blePublishLog("L%.0f R%.0f", cfgClb.lcForceCalibFctL, cfgClb.lcForceCalibFctR);
    return 0;
  case REQ_CALIB_APPLY:
    cmd = CALIB_APPLY;
    break;
  case REQ_CALIB_VERIFY:
    cmd = CALIB_VERIFY;
    break;
  case REQ_CALIB_PERSIST:
    cmd = CALIB_PERSIST;
    break;
  case REQ_SET_CR:
    cfgRt.crankRadius = arg1 < 1? arg1 * 1000 : arg1;
    DPS_CRANK_CIRC_VEL_MULTI =  PI * cfgRt.crankRadius / 1000.f / 180.f;
    printActiveCfg();
    return 0;
  case REQ_SET_ED:
    cfgRt.pwrExpDecFctr = arg1 > 1? arg1 / 100 : arg1;
    printActiveCfg();
    return 0;
  case REQ_SET_PAR:
    cfgRt.pwrAvgRevs = arg1;
    printActiveCfg();
    return 0;
  case REQ_SET_IPM:
    cfgRt.instPwrMeas = arg1;
    printActiveCfg();
    return 0;
  case REQ_GET_CR:
    blePublishLog("CR: %d", cfgRt.crankRadius);
    return 0;
  case REQ_GET_ED:
    blePublishLog("ED: %.2f", cfgRt.pwrExpDecFctr);
    return 0;
  case REQ_GET_PAR:
    blePublishLog("PAR: %d", cfgRt.pwrAvgRevs);
    return 0;
  case REQ_GET_IPM:
    blePublishLog("IPM: %d", cfgRt.instPwrMeas);
    return 0;
  case REQ_LOG_MODE:
    bleLogMode = arg1;
    return 0;
  default:
    return 0;
  }
  syncCmdExec(cmd, 3000);
  return 0;
}

inline void systemOff() {
  cfgClose();
  disableSD();
  //sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  //sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
  //__WFE();
  //__WFI();
  //sd_app_evt_wait() ; //SOFT DEVICE “SYSTEM ON” POWER SAVE
  sd_power_system_off();
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  // blink once to confirm startup
  blinkLED(1);

  applyDefaultRuntimeCfg();

  cfgSetup();
  cfgRead(cfgClb, cfgRt);
  cfgClose();

  DPS_CRANK_CIRC_VEL_MULTI =  PI * cfgRt.crankRadius / 1000.f / 180.f;
  
  lcSetup();
  lcGetAvailableLCs(lcActiveL, lcActiveR);
  lcSetScaleFactor(cfgClb.lcForceCalibFctL, cfgClb.lcForceCalibFctR);

  lcDoOffsetCalib(LOAD_CELL_EXP_SPS_RATE);
  lcSoftPowerDown();

  gaSetup();
  bleSetup(lcActiveL, lcActiveR);

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
  bleSetCfgAndMonRequestCb(doProcessCfmRequest);

  // blink twice to confirm setup complete
  blinkLED(2);
  printActiveCfg();
}

void loop() {
  static uint32_t idleCtr = 0;
  TickType_t loopWakeTime = xTaskGetTickCount();

  static int clbMode = CALIB_NOT_ACTIVE;
  static float clbMaxRawAvg = 0;

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
    idleConnected = (lastRevMI.avgAngVel == 0);
    
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
        blePublishLog("OCR: L%.1f R%.1f", zForceCalibDiffL, zForceCalibDiffR);
        break;
      case LOOP_TASK_CMD::UPDATE_POWER:
        blePublishRevUpdate(lastRev);
      break;
      case LOOP_TASK_CMD::CALIB_LC_L:
      clbMode = CALIB_MODE::CALIB_L;
      clbMaxRawAvg = 0;
      break;
      case LOOP_TASK_CMD::CALIB_LC_R:
      clbMode = CALIB_MODE::CALIB_R;
      clbMaxRawAvg = 0;
      break;
      case LOOP_TASK_CMD::CALIB_APPLY:
      {
        if(clbMode == CALIB_MODE::CALIB_L) {
          cfgClb.lcForceCalibFctL = clbMaxRawAvg / lcClbForce;
        } else if (clbMode == CALIB_MODE::CALIB_R) {
          cfgClb.lcForceCalibFctR = clbMaxRawAvg / lcClbForce;
        }
        lcSetScaleFactor(cfgClb.lcForceCalibFctL, cfgClb.lcForceCalibFctR);
        printActiveCfg();
        clbMode = CALIB_MODE::CALIB_TEST;
      }
      break;
      case LOOP_TASK_CMD::CALIB_VERIFY:
      clbMode = CALIB_MODE::CALIB_TEST;
      break;
      case LOOP_TASK_CMD::CALIB_PERSIST:
      cfgSync(cfgClb);
      printActiveCfg();
      clbMode = CALIB_MODE::CALIB_NOT_ACTIVE;
      break;
      default:
        break;
      }
      xSemaphoreGive(loopTaskCmdDoneSemaphore);
    }

    doInfrequentUpdateOperations();

    if(clbMode == CALIB_MODE::CALIB_NOT_ACTIVE) {
      switch (bleLogMode)
      {
        case BLE_LOG_DISABLED:
        /* nothing to do */
        break;
      case BLE_LOG_POWER_HR:
        blePublishLog("L%.0f R%.0f C%.0f S%d", lastRev.powerL, lastRev.powerR, lastRevMI.avgAngVel / 6, lastRevMI.numOfSmpls);
        break;
      default:
        break;
      }
    } else if (clbMode == CALIB_MODE::CALIB_TEST) {
      blePublishLog("CT: L%.1f R%.1f", lastRevStats.avgRawValueL / cfgClb.lcForceCalibFctL, lastRevStats.avgRawValueR / cfgClb.lcForceCalibFctR);
    } else {
      char mC = 'L';
      float avgRawValue = 0;
      if(clbMode == CALIB_MODE::CALIB_L) {
        avgRawValue = lastRevStats.avgRawValueL;
      } else if (clbMode == CALIB_MODE::CALIB_R) {
        avgRawValue = lastRevStats.avgRawValueR;
        mC = 'R';
      }
      if (avgRawValue > clbMaxRawAvg) {
        clbMaxRawAvg = avgRawValue;
      }
      blePublishLog("C%c: %.3f", mC, clbMaxRawAvg);
    }

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
      cfgSync(cfgRt);
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
