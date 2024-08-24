#include <Arduino.h>
#include <bluefruit.h>
#include "ble_svcs.h"

#include "ble_ext_svcs_config.h"

#ifdef ENABLE_EXT_SVCS
#include "ble_ext_svcs.h"
#endif

#define DEBUG

#ifndef DEV_NAME
#define DEV_NAME "OBPM"
#endif

#ifndef DEV_REV
#define DEV_REV  "01"
#endif

#ifndef DEV_OP_MODE
char devOpModeStrLR[4] = "-";
char devOpModeStrL[4] = "L-";
char devOpModeStrR[4] = "R-";
char* devOpModeStr = nullptr;
#else
char devOpModeStr[4] = DEV_OP_MODE;
#endif

#define BLE_ADV_TIMEOUT_S 0
#define BLE_FAST_ADV_TIMEOUT_S 300

#define BLE_FAST_ADV_INTVL_MS  1 * 1000
#define BLE_SLOW_ADV_INTVL_MS 10 * 1000

#define MAX_CONNECTIONS 3

BLEDis     bleDIS; // default "Device Information Service"
BLEBas     bleBS;  // default "Battery Service"
BLEService pwrSvc;
BLECharacteristic pwrMeasChr, pwrFeatChr, pwrCtlChr, sensLocChr;

uint32_t pwrFeatures = FTR_CRANK_REV | FTR_OFS_CALIB /*| FTR_P_PWR_BAL*/;

bleOffsetCompCb_t* offsetCompCallback = nullptr;
bleCfmReqCb_t* cfmReqCallback = nullptr;

bool pwrLeftPresent  = false;
bool pwrRightPresent = false;

/*
 * Only here for development.
 */
BLEService        cfmService = BLEService(0xcafe);
BLECharacteristic cfgCtrlChr = BLECharacteristic("35916a45-9726-4ef4-b09d-f3284968f03c");
BLECharacteristic  cfgMonChr = BLECharacteristic("5abc3692-fca4-4a69-955d-cd0442de273f");


std::set<uint16_t> availConnHndls;

void bleConnectCb(uint16_t connHndl)
{
  BLEConnection* connection = Bluefruit.Connection(connHndl);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print(F("Central connected: "));
  Serial.println(central_name);
  
#ifdef ENABLE_EXT_SVCS
  bleExtAddConnection(connHndl);
#endif

  availConnHndls.insert(connHndl);

  if (availConnHndls.size() < MAX_CONNECTIONS) {
    // keep advertising active
    Bluefruit.Advertising.start();
  }
}

void bleDisconnectCb(uint16_t connHndl, uint8_t reason)
{
  (void) reason;

  availConnHndls.erase(connHndl);

#ifdef ENABLE_EXT_SVCS
  bleExtRemoveConnection(connHndl);
#endif

  Serial.println();
  Serial.println(F("Central disconnected."));
}

void respondWithWrongInput() {
  if (cfmReqCallback) {
    cfmReqCallback(REQ_LOG_MODE, nullptr, 0, 0, 0);
  }
  blePublishLog("Wrong input");
}

void bleWriteCb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t *data, uint16_t len) {
#ifdef DEBUG
  Serial.print(F("Writte to: ")); Serial.println(chr->uuid.toString());
  Serial.printf("Written data len: %d\n", len);
  Serial.printf("Written data: %d\n", data[0]);
#endif
  if (chr->uuid == pwrCtlChr.uuid) {
    if (len == 1 && data[0] == 0x0C) {
      uint8_t resp[5] = {0x20, 0x0C, 0x00};
      bool resultValid = false;
      int16_t result = offsetCompCallback ? offsetCompCallback(resultValid): 0;
      if (resultValid) {
        resp[2] = 0x01;
        *(int16_t*)(&resp[3]) = result;
      }
      chr->indicate(conn_hdl, resp, sizeof(resp));
    }
  }

  if (chr->uuid == cfgCtrlChr.uuid) {
    if(!cfmReqCallback) {
      return;
    }
    if (data[0] >= 'A' && data[0] <= 'Z') {
      switch (data[0])
      {
      case 'C':
        if(data[1] == 'R' || data[1] == 'L') {
          data[len] = '\0';
          if (data[len - 1] < '0' || data[len - 1] > '9') {
            respondWithWrongInput();
            break;
          }
          float force = -1;
          if(data[2] == 'S') {
            float fct = atof((char*)&data[3]);
            cfmReqCallback(data[1] == 'R' ? REQ_CALIB_SET_R : REQ_CALIB_SET_L, nullptr, fct, 0, 0);
          } else {
            if (data[2] == 'W') {
              force = atof((char*)&data[3]);
              force *= 9.80928f;
            } else if (data[2] == 'F') {
              force = atof((char*)&data[3]);
            } else {
              respondWithWrongInput();
              break;
            }
            cfmReqCallback(data[1] == 'R' ? REQ_CALIB_AUTO_R : REQ_CALIB_AUTO_L, nullptr, force, 0, 0);
          }
        } else if(data[1] == 'A' ) {
          cfmReqCallback(REQ_CALIB_APPLY, nullptr, 0, 0, 0);
        } else if(data[1] == 'V' ) {
          cfmReqCallback(REQ_CALIB_VERIFY, nullptr, 0, 0, 0);
        } else if(data[1] == 'P' ) {
          cfmReqCallback(REQ_CALIB_PERSIST, nullptr, 0, 0, 0);
        } else if(data[1] == 'G' ) {
          cfmReqCallback(REQ_CALIB_GET, nullptr, 0, 0, 0);
        } else {
          respondWithWrongInput();
          break;
        }
        break;
      case 'O':
        if(offsetCompCallback && data[1] == 'C') {
          bool resultValid = false;
          offsetCompCallback(resultValid);
        } else {
          respondWithWrongInput();
          break;
        }
        break;
      case 'L':
        if(data[1] == 'M') {
          data[len] = '\0';
          cfmReqCallback(REQ_LOG_MODE, nullptr, atof((char*)&data[2]), 0, 0);
        }
        break;
      case 'G':
      case 'S':
        {
          uint8_t* pArg = nullptr;
          cfmReq_t req = REQ_INVALID;
          if(data[1] == 'C' && data[2] == 'R') {
            req = data[0] == 'S' ? REQ_SET_CR : REQ_GET_CR;
            pArg = &data[3];
          } else if(data[1] == 'E' && data[2] == 'D') {
            req = data[0] == 'S' ? REQ_SET_ED : REQ_GET_ED;
            pArg = &data[3];
          } else if(data[1] == 'P' && data[2] == 'A' && data[3] == 'R') {
            req = data[0] == 'S' ? REQ_SET_PAR : REQ_GET_PAR;
            pArg = &data[4];
          } else if(data[1] == 'I' && data[2] == 'P' && data[3] == 'M') {
            req = data[0] == 'S' ? REQ_SET_IPM : REQ_GET_IPM;
            pArg = &data[4];
          } else {
            respondWithWrongInput();
            break;
          }
          if (data[0] == 'S') {
            data[len] = '\0';
            if (data[len - 1] < '0' || data[len - 1] > '9') {
              respondWithWrongInput();
              break;
            }
            float arg = atof((char*)pArg);
            cfmReqCallback(req, nullptr, arg, 0, 0);
          } else {
            cfmReqCallback(req, nullptr, 0, 0, 0);
          }
        }
        break;
      default:
        respondWithWrongInput();
        break;
      }
    } else {
      //TODO: finish this...
      uint8_t resp[20];
      uint8_t respLen = cfmReqCallback ? cfmReqCallback((cfmReq_t)data[0], resp, *(float*)&data[1], *(float*)&data[5], *(float*)&data[9]): 0;
      if(respLen > 0) {
        chr->indicate(conn_hdl, resp, respLen);
      }
    }
  }
}

void bleCccdCb(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t value) {
#ifdef DEBUG
  // Display the raw request packet
    Serial.printf("CCCD Updated: %d\n", value);

  if (chr->uuid == pwrMeasChr.uuid) {
    if (chr->notifyEnabled()) {
      Serial.println(F("Pwr Measurement 'Notify' enabled"));
    } else {
      Serial.println(F("Pwr Measurement 'Notify' disabled"));
    }
  }
  if (chr->uuid == pwrCtlChr.uuid) {
    if (chr->indicateEnabled()) {
      Serial.println(F("Pwr Control 'Indicate' enabled"));
    } else {
      Serial.println(F("Pwr Control 'Indicate' disabled"));
    }
  }
  if (chr->uuid == cfgCtrlChr.uuid) {
    if (chr->indicateEnabled()) {
      Serial.println(F("Cfg 'Indicate' enabled"));
    } else {
      Serial.println(F("Cfg 'Indicate' disabled"));
    }
  }
#endif
}

void bleSetOffsetCompensationCb(bleOffsetCompCb_t* bleOffsetCompCb) {
  offsetCompCallback = bleOffsetCompCb;
}

void bleSetCfgAndMonRequestCb(bleCfmReqCb_t* bleCfmRequestCb) {
  cfmReqCallback = bleCfmRequestCb;
}

/*
 * Only here for debug log purposes
 */
void bleSetupCFMSvc() {
  cfmService.begin();
#ifdef ENABLE_BLE_LOG
  cfgMonChr.setProperties(CHR_PROPS_NOTIFY);
  cfgMonChr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  cfgMonChr.setMaxLen(20);
  cfgMonChr.begin();
#endif
  // Has nothing to do with any spec.
  cfgCtrlChr.setProperties(CHR_PROPS_INDICATE | CHR_PROPS_WRITE);
  cfgCtrlChr.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  cfgCtrlChr.setMaxLen(20);
  cfgCtrlChr.setCccdWriteCallback(bleCccdCb);
  cfgCtrlChr.setWriteCallback(bleWriteCb);
  cfgCtrlChr.begin();
}

void bleNotifyPwrMeas(int16_t instPwrL, int16_t instPwrR, uint16_t crankRevs, long time) {
  uint16_t instPwr = 0;
  
  if (pwrLeftPresent && pwrRightPresent) {
    instPwr = instPwrL + instPwrR;
  } else if (pwrLeftPresent) {
    instPwr = instPwrL * 2;
  } else if (pwrRightPresent) {
    instPwr = instPwrR * 2;
  }

  uint16_t timeUpdate = uint16_t(time / 1000.f * 1024.f) % 65536;
  uint8_t pwrMeasData[20] = { 0, 0, LMOCS(instPwr) };
  uint8_t oIdx = 4;
  uint16_t flags = 0;

  if (pwrFeatures & FTR_P_PWR_BAL) {
    uint8_t ppb = 100;
    float pwrDiff = instPwrR - instPwrL;
    if (pwrDiff != 0) {
      ppb -= uint8_t(instPwr / pwrDiff);
    }
    pwrMeasData[oIdx] = ppb;
    oIdx++;

    flags |= 0x03;
  }

  if (pwrFeatures & FTR_CRANK_REV) {
    pwrMeasData[oIdx] = LSO(crankRevs);
    oIdx++;
    pwrMeasData[oIdx] = MSO(crankRevs);
    oIdx++;
    pwrMeasData[oIdx] = LSO(timeUpdate);
    oIdx++;
    pwrMeasData[oIdx] = MSO(timeUpdate);
    oIdx++;

    flags |= 0x20;
  }

  pwrMeasData[0] = LSO(flags);
  pwrMeasData[1] = MSO(flags);

  for(uint16_t connHndl : availConnHndls) {
    if(pwrMeasChr.notifyEnabled(connHndl))
      pwrMeasChr.notify(connHndl, pwrMeasData, oIdx);
  }
}

void blePublishRevUpdate(revUpdt_t& revUpdate) {
  bleNotifyPwrMeas(revUpdate.powerL, revUpdate.powerR, revUpdate.crankRev, revUpdate.time);

#ifdef ENABLE_EXT_SVCS
  notifyExtTePs(revUpdate.teL, revUpdate.teR, revUpdate.psL, revUpdate.psR);
#endif
}

void blePublishBatSOC(uint8_t batSOC) {
  bleBS.write(batSOC);
#ifdef DEBUG
  Serial.printf("Updated battery SOC to %d\n", batSOC);
#endif
}

#ifdef ENABLE_BLE_LOG
void blePublishLog(const char* fmt, ...) {
  static char msg[20];
  char* lEntry = nullptr;
  int nChar = 0;

  for(uint16_t connHndl : availConnHndls) {
    if(cfgMonChr.notifyEnabled(connHndl)) {
      if (lEntry == nullptr) {
        va_list args;
        va_start(args, fmt);
        nChar = vsnprintf(msg, sizeof(msg), fmt, args);
        va_end(args);

        if (nChar < 0) {
          Serial.println(F("Failed to create log entry."));
          return;
        }
        lEntry = msg;
      }

      cfgMonChr.notify(connHndl, lEntry, nChar);
    }
  }
}
#endif

void bleSetupPwrSvc() {
  uint16_t measLen = 4;

  if (pwrLeftPresent & pwrRightPresent) {
    pwrFeatures |= FTR_P_PWR_BAL;
  }
  if (pwrFeatures & FTR_P_PWR_BAL) {
    measLen += 1;
  }
  if (pwrFeatures & FTR_CRANK_REV) {
    measLen +=4;
  }

  pwrSvc.setUuid(UUID16_SVC_CYCLING_POWER);
  pwrSvc.begin();
  
  pwrMeasChr.setUuid(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
  pwrMeasChr.setFixedLen(measLen);
  pwrMeasChr.setProperties(CHR_PROPS_NOTIFY);
  pwrMeasChr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pwrMeasChr.setCccdWriteCallback(bleCccdCb);
  pwrMeasChr.begin();
  
  pwrFeatChr.setUuid(UUID16_CHR_CYCLING_POWER_FEATURE);
  pwrFeatChr.setProperties(CHR_PROPS_READ);
  pwrFeatChr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pwrFeatChr.setFixedLen(4);
  pwrFeatChr.begin();
  pwrFeatChr.write32(pwrFeatures);

  sensLocChr.setUuid(UUID16_CHR_SENSOR_LOCATION);
  sensLocChr.setProperties(CHR_PROPS_READ);
  sensLocChr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  sensLocChr.setFixedLen(1);
  sensLocChr.begin();
  if (pwrLeftPresent && pwrRightPresent) {
    sensLocChr.write8(0);
  } else if (pwrLeftPresent) {
    sensLocChr.write8(LOC_LEFT_CRANK);
  } else if (pwrRightPresent) {
    sensLocChr.write8(LOC_RIGHT_CRANK);
  }

  if(pwrFeatures & FTR_OFS_CALIB) {
    pwrCtlChr.setUuid(UUID16_CHR_CYCLING_POWER_CONTROL_POINT);
    pwrCtlChr.setProperties(CHR_PROPS_INDICATE | CHR_PROPS_WRITE);
    pwrCtlChr.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    pwrCtlChr.setWriteCallback(bleWriteCb);
    pwrCtlChr.setCccdWriteCallback(bleCccdCb);
    pwrCtlChr.begin();
  }
}

void bleSetup(bool leftCrankPowerAvailable, bool rightCrankPowerAvailable) {
#ifndef DEV_OP_MODE
  if (leftCrankPowerAvailable & rightCrankPowerAvailable) {
    devOpModeStr = devOpModeStrLR;
  } else if (leftCrankPowerAvailable) {
    devOpModeStr = devOpModeStrL;
  } else if (rightCrankPowerAvailable) {
    devOpModeStr = devOpModeStrR;
  }
#endif

  pwrLeftPresent = leftCrankPowerAvailable;
  pwrRightPresent = rightCrankPowerAvailable;

  char buf[20];
  Bluefruit.begin(MAX_CONNECTIONS);
  ble_gap_addr_t gapaddr = Bluefruit.getAddr();
  uint32_t devNum = *(uint16_t*)&gapaddr.addr[0] + (*(uint16_t*)&gapaddr.addr[2] & 0xfff << 3);
  sprintf(buf, "%s%s%s%05ld", DEV_NAME, DEV_REV, devOpModeStr, devNum);
  Bluefruit.setName(buf);

  Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(-4);
  Bluefruit.Periph.setConnectCallback(bleConnectCb);
  Bluefruit.Periph.setDisconnectCallback(bleDisconnectCb);

  // Configure and Start the Device Information Service
  bleDIS.setManufacturer("QuSensorIndustries");
  sprintf(buf, "%s rev%s", DEV_NAME, DEV_REV);
  bleDIS.setModel(buf);

  bleDIS.begin();
  bleBS.begin();

  bleSetupPwrSvc();

  // just for debug
  bleSetupCFMSvc();

#ifdef ENABLE_EXT_SVCS
  bleExtSetupSvcs();
#endif

  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.setIntervalMS(BLE_FAST_ADV_INTVL_MS, BLE_SLOW_ADV_INTVL_MS);
  Bluefruit.Advertising.setFastTimeout(BLE_FAST_ADV_TIMEOUT_S);
  Bluefruit.Advertising.addUuid(UUID16_SVC_CYCLING_POWER);
  
#ifdef ENABLE_EXT_SVCS
  bleExtAddAdvSvc(&Bluefruit.Advertising);
#endif

  Bluefruit.Advertising.start();

#ifdef DEBUG
  Serial.println(F("All BLE services are configured. Starting advertising..."));
#endif
}
