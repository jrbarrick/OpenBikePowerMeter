#include <Arduino.h>
#include <bluefruit.h>
#include "ble_svcs.h"
#include <set>

//#define ENABLE_EXT_SVCS

#ifdef ENABLE_EXT_SVCS
#include "ble_ext_svcs.h"
#endif

#define DEBUG
#ifndef DEV_NAME
#define DEV_NAME "QuPwrMeter"
#endif


#define BLE_ADV_TIMEOUT_S 0
#define BLE_FAST_ADV_TIMEOUT_S 60

#define BLE_FAST_ADV_INTVL_MS  1 * 1000
#define BLE_SLOW_ADV_INTVL_MS 10 * 1000

#define MAX_CONNECTIONS 3
//#define NOTIFY_PEDAL_POWER_BALANCE

BLEService        pwrService  = BLEService(UUID16_SVC_CYCLING_POWER);
BLECharacteristic pwrMeasChar = BLECharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
BLECharacteristic pwrFeatChar = BLECharacteristic(UUID16_CHR_CYCLING_POWER_FEATURE);
BLECharacteristic pwrCtlChar  = BLECharacteristic(UUID16_CHR_CYCLING_POWER_CONTROL_POINT);
BLECharacteristic pwrLocChar  = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);

bleOffsetCompCb_t* offsetCompCallback = nullptr;

BLEDis bleDIS;   // default "Device Information Service"
BLEBas bleBS;    // default "Battery Service"


/*
 * Only here for development.
 */
BLEService        cfmService = BLEService(0xcafe);
BLECharacteristic cfgChar    = BLECharacteristic("35916a45-9726-4ef4-b09d-f3284968f03c");
BLECharacteristic logChar    = BLECharacteristic("5abc3692-fca4-4a69-955d-cd0442de273f");

std::set<uint16_t> availConnHndls;
uint8_t mainAdvData[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
uint8_t mainAdvSize;

void bleStartAdvertising() {
  //Bluefruit.Advertising.stop();
  Bluefruit.Advertising.setData(mainAdvData, mainAdvSize);

  Bluefruit.Advertising.addService(pwrService);
#ifdef ENABLE_BLE_LOG
  //Bluefruit.Advertising.addService(cfmService);
#endif
#ifdef ENABLE_EXT_SVCS
  bleExtAddAdvSvc(&Bluefruit.Advertising);
#endif

  Bluefruit.Advertising.start(0);
}

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
    bleStartAdvertising();
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
  Serial.println("Central disconnected.");
  if (availConnHndls.size() < MAX_CONNECTIONS) {
    // restart advertising if any connected dev. disconnects
    bleStartAdvertising();
  }
}

void blePrepareAdvertising(void) {
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.setIntervalMS(BLE_FAST_ADV_INTVL_MS, BLE_SLOW_ADV_INTVL_MS);
  Bluefruit.Advertising.setFastTimeout(BLE_FAST_ADV_TIMEOUT_S);

  memcpy(mainAdvData, Bluefruit.Advertising.getData(), BLE_GAP_ADV_SET_DATA_SIZE_MAX);
  mainAdvSize = Bluefruit.Advertising.count();
}

void bleWriteCb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t *data, uint16_t len) {
#ifdef DEBUG
  Serial.print("Writte to: "); Serial.println(chr->uuid.toString());
  Serial.printf("Written data len: %d\n", len);
  Serial.printf("Written data: %d\n", data[0]);
#endif
  if (chr->uuid == pwrCtlChar.uuid) {
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

  if (chr->uuid == pwrMeasChar.uuid) {
    if (chr->notifyEnabled()) {
      Serial.println("Pwr Measurement 'Notify' enabled");
    } else {
      Serial.println("Pwr Measurement 'Notify' disabled");
    }
  }
  if (chr->uuid == pwrCtlChar.uuid) {
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

void bleSetupPwrSvc(void) {
  pwrService.begin();

  uint16_t measLen = 8;
#ifdef NOTIFY_PEDAL_POWER_BALANCE
  measLen++;
#endif
  pwrMeasChar.setFixedLen(measLen);
  pwrMeasChar.setProperties(CHR_PROPS_NOTIFY);
  pwrMeasChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pwrMeasChar.setCccdWriteCallback(bleCccdCb);
  pwrMeasChar.begin();
  
  uint32_t pwrFeatures = 0;
  pwrFeatChar.setProperties(CHR_PROPS_READ);
  pwrFeatChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pwrFeatChar.setFixedLen(4);
  pwrFeatChar.begin();
  
#ifdef NOTIFY_PEDAL_POWER_BALANCE
  pwrFeatures |=0x0001; // set bit 3 because pedal power balance  will be supported 
#endif
  pwrFeatures |=0x0008; // set bit 3 because crank revolutions will be present
  pwrFeatures |=0x0200; // set bit 9 because offset calibration will be supported
  
  //pwrFeatures |=0x0800; // set bit 11 because multiple sensor locations will be supported
  pwrFeatChar.write32(pwrFeatures);

  pwrLocChar.setProperties(CHR_PROPS_READ);
  pwrLocChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pwrLocChar.setFixedLen(1);
  pwrLocChar.begin();
  pwrLocChar.write8(5); // setting to "left crank"

  pwrCtlChar.setProperties(CHR_PROPS_INDICATE | CHR_PROPS_WRITE);
  pwrCtlChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  pwrCtlChar.setWriteCallback(bleWriteCb);
  pwrCtlChar.setCccdWriteCallback(bleCccdCb);
  pwrCtlChar.begin();
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
    if(pwrMeasChar.notifyEnabled(connHndl))
      pwrMeasChar.notify(connHndl, pwrMeasData, sizeof(pwrMeasData));
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

void bleSetup() {
  Bluefruit.begin(MAX_CONNECTIONS);
  Bluefruit.setName(DEV_NAME);
  Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(-4);
  Bluefruit.Periph.setConnectCallback(bleConnectCb);
  Bluefruit.Periph.setDisconnectCallback(bleDisconnectCb);

  // Configure and Start the Device Information Service
  bleDIS.setManufacturer("QuSensorIndustries");
  bleDIS.setModel("BFP rev1");

  bleDIS.begin();
  bleBS.begin();

  bleSetupPwrSvc();

  // just for debug
  bleSetupCFMSvc();

#ifdef ENABLE_EXT_SVCS
  bleExtSetupSvcs();
#endif
  blePrepareAdvertising();
  bleStartAdvertising();

#ifdef DEBUG
  Serial.println("All BLE services are configured. Starting advertising...");
#endif
}
