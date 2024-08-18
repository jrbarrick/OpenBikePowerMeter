#include <Arduino.h>
#include <bluefruit.h>
#include "ble_svcs.h"

#define ENABLE_EXT_SVCS

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
#define DEV_OP_MODE  "-" 
//#define DEV_OP_MODE  "L-"
//#define DEV_OP_MODE  "R-"
#endif

char devOpModeStr[4] = DEV_OP_MODE;

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

/*
 * Only here for development.
 */
BLEService     cfmService = BLEService(0xcafe);
BLECharacteristic cfgChar = BLECharacteristic("35916a45-9726-4ef4-b09d-f3284968f03c");
BLECharacteristic logChar = BLECharacteristic("5abc3692-fca4-4a69-955d-cd0442de273f");


std::set<uint16_t> availConnHndls;

void bleConnectCb(uint16_t connHndl)
{
  BLEConnection* connection = Bluefruit.Connection(connHndl);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Central connected: ");
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
  
  //blePwrSvc.removeConnection(connHndl);

#ifdef ENABLE_EXT_SVCS
  bleExtRemoveConnection(connHndl);
#endif

  Serial.println();
  Serial.println("Central disconnected.");
}

void bleWriteCb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t *data, uint16_t len) {
#ifdef DEBUG
  Serial.print("Writte to: "); Serial.println(chr->uuid.toString());
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
}

void bleCccdCb(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t value) {
#ifdef DEBUG
  // Display the raw request packet
    Serial.printf("CCCD Updated: %d\n", value);

  if (chr->uuid == pwrMeasChr.uuid) {
    if (chr->notifyEnabled()) {
      Serial.println("Pwr Measurement 'Notify' enabled");
    } else {
      Serial.println("Pwr Measurement 'Notify' disabled");
    }
  }
  if (chr->uuid == pwrMeasChr.uuid) {
    if (chr->indicateEnabled()) {
      Serial.println("Pwr Control 'Indicate' enabled");
    } else {
      Serial.println("Pwr Control 'Indicate' disabled");
    }
  }
#endif
}

void bleSetOffsetCompensationCb(bleOffsetCompCb_t* bleOffsetCompCb) {
  offsetCompCallback = bleOffsetCompCb;
}

/*
 * Only here for debug log purposes
 */
void bleSetupCFMSvc() {
  cfmService.begin();
#ifdef ENABLE_BLE_LOG
  logChar.setProperties(CHR_PROPS_NOTIFY);
  logChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  logChar.setMaxLen(20);
  logChar.begin();
#endif
  // Has nothing to do with any spec.
  cfgChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  // First param is the read permission, second is write.
  cfgChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  // Payload is quite limited in BLE, so come up with good logging shorthand.
  cfgChar.setMaxLen(20);
  cfgChar.setWriteCallback(bleWriteCb);
  cfgChar.begin();
}

void bleNotifyPwrMeas(int16_t instPwrL, int16_t instPwrR, uint16_t crankRevs, long time) {
  uint16_t instPwr = instPwrL + instPwrR;
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
/*
void bleNotifyPwrMeas(int16_t instPwrL, int16_t instPwrR, uint16_t crankRevs, long time) {
  uint16_t instPwr = instPwrL + instPwrR;
  uint16_t timeUpdate = uint16_t(time / 1000.f * 1024.f) % 65536;
#ifdef NOTIFY_PEDAL_POWER_BALANCE
  uint8_t ppb = 100;
  float pwrDiff = instPwrR - instPwrL;
  if (pwrDiff != 0) {
    ppb -= uint8_t(instPwr / pwrDiff);
  }
  uint8_t pwrMeasData[9] = { LMOCS(0x23), LMOCS(instPwr), ppb, LMOCS(crankRevs), LMOCS(timeUpdate) };
#else
  uint8_t pwrMeasData[8] = { LMOCS(0x20), LMOCS(instPwr), LMOCS(crankRevs), LMOCS(timeUpdate) };
#endif
  for(uint16_t connHndl : availConnHndls) {
    if(pwrMeasChr.notifyEnabled(connHndl))
      pwrMeasChr.notify(connHndl, pwrMeasData, sizeof(pwrMeasData));
  }
}
*/
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

  va_list args;
  va_start(args, fmt);
  int nChar = vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);

  if (nChar < 0) {
    Serial.println("Failed to create log entry.");
  }
  
  for(uint16_t connHndl : availConnHndls) {
    if(logChar.notifyEnabled(connHndl)) {
      logChar.notify(connHndl, msg, nChar);
    }
  }
}
#endif


void bleSetupPwrSvc() {
  pwrSvc.setUuid(UUID16_SVC_CYCLING_POWER);
  pwrSvc.begin();

  uint16_t measLen = 4;
  if (pwrFeatures & FTR_P_PWR_BAL) {
    measLen += 1;
  }
  if (pwrFeatures & FTR_CRANK_REV) {
    measLen +=4;
  }
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
  switch (devOpModeStr[0])
  {
  case 'L':
    sensLocChr.write8(LOC_LEFT_CRANK);
    break;
  case 'R':
    sensLocChr.write8(LOC_RIGHT_CRANK);
    break;
  default:
    sensLocChr.write8(0);
    break;
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

void bleSetup() {
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

  //blePrepareAdvertising();
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
  //bleStartAdvertising();

#ifdef DEBUG
  Serial.println("All BLE services are configured. Starting advertising...");
#endif
}
