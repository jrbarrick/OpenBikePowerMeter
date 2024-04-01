
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
#define MAX_UPDATE_INTRVL 2500

#define NO_OP_TOUT 10
#define NO_OP_SYSTEM_OFF_TOUT 60
#define PEDALING_DET_SMPLS_THRSHLD  3

//#define APPLY_EXP_DECAY_ON_DPS_FCTR 0.9
//#define APPLY_EXP_DECAY_ON_FORCE_FCTR 0.9
#define APPLY_EXP_DECAY_ON_POWER_FCTR 0.85

const float DPS_MIN = 1000.f * (360.f / MAX_UPDATE_INTRVL);
const float DPS_CRANK_CIRC_VEL_MULTI =  PI * CRANK_RADIUS / 180;

uint16_t totalCrankRevs = 0;

bool idleConnected = false;

TaskHandle_t loopTaskHandle = NULL;
SemaphoreHandle_t loopTaskCmdDoneSemaphore = NULL;

enum LOOP_TASK_CMD {
  CALIBRATE_OFFSETS = 1
};
float zForceCalibDiffL = 0, zForceCalibDiffR = 0;

#ifdef COLLECT_REV_STATS
struct {
  float avgAngVel = 0;
  float avgForceL = 0;
  float avgForceR = 0;
  float avgForcePosL = 0;
  float avgForcePosR = 0;
  float avgRawValueL = 0;
  float avgRawValueR = 0;
  float circVel = 0;
  int16_t numOfSmpls = 0;
} lastRevStats;
#endif

struct {
  long  time = 0;
  float cad = 0;
  float powerL = 0;
  float powerR = 0;
  float teL = 0;
  float teR = 0;
  float psL = 0;
  float psR = 0;
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
inline void writeRideLogEntryToSD(const long& timeNow, const int16_t& power = 0, const float& cad = 0, const float& teL = 0, const float& psL = 0, const float& teR = 0, const float& psR = 0, const double& avgForceL = 0, const double& avgForceR = 0, const float& avgMPS = 0, const int16_t numOfSmpls = 0)  {
  if (sdCardDetected) {
    static char msg[1024];
    static bool initLog = true;
    if (initLog) {
      #ifdef COLLECT_REV_STATS
      sprintf(msg, "Time; Cad; Power; Pedal; avgForceL; avgForceR; avgMPS; TEL; PSL; TER; PSR; Smpls");
      #else
      sprintf(msg, "Time; Cad; Power; Pedal; TEL; PSL; TER; PSR");
      #endif
      SD.mkdir(RIDE_PATH_PFX);
      contRideLogFile = SD.open(RIDE_PATH_PFX + "ride" + String(START_IDX) +".csv", FILE_WRITE);
      contRideLogFile.println(msg);
      initLog = false;
    }
    #ifdef COLLECT_REV_STATS
    sprintf(msg, "%ld; %.3f; %d; %d; %.3f; %.3f; %.3f; %.3f; %.3f; %.3f; %.3f; %d", timeNow, cad, power, cad > 0, teL, psL, teR, psR, avgForceL, avgForceR, avgMPS, numOfSmpls);
    #else
    sprintf(msg, "%ld; %.3f; %d; %d; %.3f; %.3f; %.3f; %.3f", timeNow, cad, power, cad > 0, teL, psL, teR, psR);
    #endif
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
#ifdef COLLECT_REV_STATS
inline void writeRideLogEntryToSD(const long& timeNow, const int16_t& power = 0, const float& cad = 0, const float& teL = 0, const float& psL = 0, const float& teR = 0, const float& psR = 0,  const double& avgForceL = 0, const double& avgForceR = 0, const float& avgMPS = 0, const int16_t numOfSmpls = 0) {}
#else
inline void writeRideLogEntryToSD(const long& timeNow, const int16_t& power = 0, const float& cad = 0, const float& teL = 0, const float& psL = 0, const float& teR = 0, const float& psR = 0) {}
#endif
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

inline bool checkPedalingAndUpdateInterval(long& timeNow, float& angVel, bool& pedaling) {
  static long lastUpdate = millis();
  static uint32_t instPedlCtr = 0;
  //static bool prevRevPedaling = false;

  bool instPedaling = (angVel > DPS_MIN);

  if (instPedaling) {
    instPedlCtr++;
  } else {
    instPedlCtr = 0;
    //prevRevPedaling = false;
  }
  pedaling = instPedlCtr > PEDALING_DET_SMPLS_THRSHLD;

  if ((timeNow - lastUpdate) < (pedaling ? 1000.f * (360.f / angVel) : MAX_UPDATE_INTRVL)) {
    return false;
  }
  /*
  if (!prevRevPedaling && pedaling
      && maxForceL > PEDALING_SYNC_FORCE_THRSHLD
      && forceL < maxForceL) {
    resetInterval();
    return false;
  }
  prevRevPedaling = pedaling;
  */
  lastUpdate = timeNow;
  return true;
}

inline void applyExpDecay(float & currValue, float & avgValue, const float& avgWeight) {
  avgValue = currValue * avgWeight + (avgValue * (1 - avgWeight));
}

void isrCbLoadUpdate(float& instForceL, float& instForceR, int32_t& valueL, int32_t& valueR) {
  static float maxForceL = FLT_MIN;
  static float maxForceR = FLT_MIN;
  static float minForceR = FLT_MAX;
  static float minForceL = FLT_MAX;

  static int16_t numOfPosForceValsL = 0;
  static int16_t numOfPosForceValsR = 0;
  static int16_t numOfSmpls = 0;

  static double sumAngVel = 0.f;
  static double sumForceL = 0.f;
  static double sumForceR = 0.f;
  static double sumForcePosL = 0.f;
  static double sumForcePosR = 0.f;

  #ifdef COLLECT_REV_STATS
  static double sumValueL = 0.f;
  static double sumValueR = 0.f;
  #endif

  static float angVel = 0;
  static float forceL = 0;
  static float forceR = 0;

  long smplTime = millis();

  float instDPS = gaGetAngularVelocity();
  instDPS = (instDPS < 90 || instDPS > 1440) ? 0: abs(instDPS); // filter ang. vel. to allow only pos. cad. >15rpm & <240rpm
  #ifdef APPLY_EXP_DECAY_ON_DPS_FCTR
  applyExpDecay(instDPS, angVel, APPLY_EXP_DECAY_ON_DPS_FCTR);
  #else
  angVel = instDPS;
  #endif
  #ifdef APPLY_EXP_DECAY_ON_FORCE_FCTR
  applyExpDecay(instForceL, forceL, APPLY_EXP_DECAY_ON_FORCE_FCTR);
  applyExpDecay(instForceR, forceR, APPLY_EXP_DECAY_ON_FORCE_FCTR);
  #else
  forceL = instForceL;
  forceR = instForceR;
  #endif

  sumAngVel += angVel;
  sumForceL += forceL;
  sumForceR += forceR;
  #ifdef COLLECT_REV_STATS
  sumValueL += valueL;
  sumValueR += valueR;
  #endif

  if (forceL > 0) {
    sumForcePosL += forceL;
    numOfPosForceValsL++;
  }
  if (forceR > 0) {
    sumForcePosR += forceR;
    numOfPosForceValsR++;
  }
  if (forceL < minForceL) minForceL = forceL;
  if (forceL > maxForceL) maxForceL = forceL;
  if (forceR < minForceR) minForceR = forceR;
  if (forceR > maxForceR) maxForceR = forceR;

  numOfSmpls++;

  bool pedaling = false;
  if(!checkPedalingAndUpdateInterval(smplTime, angVel, pedaling))
    return;

  lastRev.time = smplTime;

  if (pedaling) {
    totalCrankRevs += 1;
    
    float avgAngVel = sumAngVel / numOfSmpls;
    float avgForceL = sumForceL / numOfSmpls;
    float avgForceR = sumForceR / numOfSmpls;

    float avgForcePosL = (numOfPosForceValsL > 1) ? sumForcePosL / numOfPosForceValsL : sumForcePosL;
    float avgForcePosR = (numOfPosForceValsR > 1) ? sumForcePosR / numOfPosForceValsR : sumForcePosR;

    lastRev.teL = avgForceL / avgForcePosL;
    lastRev.teR = avgForceR / avgForcePosR;
    lastRev.psL = avgForceL / maxForceL;
    lastRev.psR = avgForceR / maxForceR;

    // calculate circular velocity from avgAngVel
    float circVel = avgAngVel * DPS_CRANK_CIRC_VEL_MULTI;

    // calculate instantaneous power
    float instPowerL = circVel * avgForceL;
    float instPowerR = circVel * avgForceR;
    #ifdef APPLY_EXP_DECAY_ON_POWER_FCTR
    applyExpDecay(instPowerL, lastRev.powerL, APPLY_EXP_DECAY_ON_POWER_FCTR);
    applyExpDecay(instPowerR, lastRev.powerR, APPLY_EXP_DECAY_ON_POWER_FCTR);
    #else
    lastRev.powerL = instPowerL;
    lastRev.powerR = instPowerR;
    #endif

    lastRev.cad  = avgAngVel / 6;

    #ifdef COLLECT_REV_STATS
    lastRevStats.circVel   = circVel;
    lastRevStats.avgAngVel = avgAngVel;
    lastRevStats.avgForceL = avgForceL;
    lastRevStats.avgForceR = avgForceR;
    lastRevStats.avgForcePosL = avgForcePosL;
    lastRevStats.avgForcePosR = avgForcePosR;
    lastRevStats.numOfSmpls   = numOfSmpls;
    #endif
  } else {
    memset(&lastRev, 0, sizeof(lastRev));
  }

  #ifdef COLLECT_REV_STATS
  if (!pedaling) {
    memset(&lastRevStats, 0, sizeof(lastRevStats));
  }
  lastRevStats.avgRawValueL = sumValueL / numOfSmpls;
  lastRevStats.avgRawValueR = sumValueR / numOfSmpls;
  #endif

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(loopTaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
  
  maxForceL = FLT_MIN;
  maxForceR = FLT_MIN;
  minForceR = FLT_MAX;
  minForceL = FLT_MAX;
  numOfPosForceValsL = 0;
  numOfPosForceValsR = 0;
  numOfSmpls = 0;
  sumAngVel = 0.f;
  sumForceL = 0.f;
  sumForceR = 0.f;
  sumForcePosL = 0.f;
  sumForcePosR = 0.f;
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
  static int32_t idleCtr = -1;
  TickType_t loopWakeTime = xTaskGetTickCount();

  #ifdef DO_RAW_MEASURE_1S_LOOP
  storeAvgMeasurementsToSD();
  return;
  #endif

  if (Bluefruit.connected()) {
    if (idleCtr >= NO_OP_TOUT || idleCtr == -1) {
      gaDisableCycledSleep();
      lcSoftPowerUp();
    }
    if (idleCtr > 0 || idleCtr == -1) {      
      lcStartContValueUpdate(isrCbLoadUpdate);
      idleCtr = 0;
    }

    uint32_t ntfValue;
    xTaskNotifyWait(0xffffffff, 0, &ntfValue, ms2tick(5000));
    idleConnected = (lastRev.cad == 0);
    
    if (ntfValue) {
      switch (ntfValue)
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
      default:
        break;
      }
      xSemaphoreGive(loopTaskCmdDoneSemaphore);
    }

    float powerTotal = lastRev.powerL + lastRev.powerR;
    blePublishPower(powerTotal, totalCrankRevs, lastRev.time);
    blePublishTePs(lastRev.teL, lastRev.psL);
 
    doInfrequentUpdateOperations();

    #ifdef COLLECT_REV_STATS
    #ifdef ENABLE_BLE_LOG
    float avgForceLS = lastRevStats.avgForceL > lastRevStats.avgForceR ? lastRevStats.avgForceL : lastRevStats.avgForceR;
    blePublishLog("P%.0f F%.1f C%.0f", powerTotal,avgForceLS, lastRev.cad);
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
    #ifdef COLLECT_REV_STATS
    writeRideLogEntryToSD(lastRev.time, powerTotal, lastRev.cad, lastRev.teL, lastRev.psL, lastRev.teR, lastRev.psR, lastRevStats.avgForceL, lastRevStats.avgForceR, lastRevStats.circVel, lastRevStats.numOfSmpls);
    #else
    writeRideLogEntryToSD(lastRev.time, powerTotal, lastRev.cad, lastRev.teL, lastRev.psL, lastRev.teR, lastRev.psR);
    #endif
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
