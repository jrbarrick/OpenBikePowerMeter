#include <Arduino.h>
#include <bluefruit.h>
#include "ble_svcs.h"


#define ENABLE_EXT_SVCS

#ifdef ENABLE_EXT_SVCS
#include "ble_ext_svcs.h"
#endif

// Service and character constants at:
// https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/bd0747473242d5d7c58ebc67ab0aa5098db56547/libraries/Bluefruit52Lib/src/BLEUuid.h
/* Pwr Service Definitions
 * Cycling Power Service:      0x1818
 * Power Measurement Char:     0x2A63
 * Cycling Power Feature Char: 0x2A65
 * Sensor Location Char:       0x2A5D
 * Power Control Char:         0x2A66
 */

#define DEBUG
#ifndef DEV_NAME
#define DEV_NAME "QuPwrMeter"
#endif

BLEService        pwrService  = BLEService(UUID16_SVC_CYCLING_POWER);
BLECharacteristic pwrMeasChar = BLECharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
BLECharacteristic pwrFeatChar = BLECharacteristic(UUID16_CHR_CYCLING_POWER_FEATURE);
BLECharacteristic pwrCtlChar  = BLECharacteristic(UUID16_CHR_CYCLING_POWER_CONTROL_POINT);
BLECharacteristic pwrLocChar  = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);

bleOffsetCompCb_t* offsetCompCallback = nullptr;

/*
 * A made up service to help development.
 */
BLEService        cfmService = BLEService(0xcafe);
BLECharacteristic logChar    = BLECharacteristic(0x1234);
BLECharacteristic cfgChar    = BLECharacteristic(0x5678);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

// callback invoked when central connects
void connectCallback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnectCallback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.println("Disconnected");
}

void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(-4);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(pwrService);
#ifdef ENABLE_BLE_LOG
  Bluefruit.Advertising.addService(cfmService);
#endif
#ifdef ENABLE_EXT_SVCS
  addExtSvc(&Bluefruit.Advertising);
#endif
  Bluefruit.Advertising.addName();

  

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void writeCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t *data, uint16_t len) {
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
      chr->indicate(resp, sizeof(resp));
    }
  }
}

void cccdCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t value) {
#ifdef DEBUG
  // Display the raw request packet
    Serial.printf("CCCD Updated: %d\n", value);

  // Check the characteristic this CCCD update is associated with in case
  // this handler is used for multiple CCCD records.
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

/*
 * Given a 16-bit uint16_t, convert it to 2 8-bit ints, and set
 * them in the provided array. Assume the array is of correct
 * size, allocated by caller. Least-significant octet is place
 * in output array first.
 */
void uint16ToLso(uint16_t val, uint8_t* out) {
  uint8_t lso = val & 0xff;
  uint8_t mso = (val >> 8) & 0xff;
  out[0] = lso;
  out[1] = mso;
}

void bleSetOffsetCompensationCb(bleOffsetCompCb_t* bleOffsetCompCb) {
  offsetCompCallback = bleOffsetCompCb;
}
/*
 * Set up the power service
 */
void setupPwr(void) {
  // Configure supported characteristics:
  pwrService.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  // Has to have notify enabled.
  // Power measurement. This is the characteristic that really matters. See:
  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.cycling_power_measurement.xml
  pwrMeasChar.setProperties(CHR_PROPS_NOTIFY);
  // First param is the read permission, second is write.
  pwrMeasChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  // 4 total bytes, 2 16-bit values
  // pwrMeasChar.setFixedLen(4);
  // 8 total bytes, 4 16-bit values
  pwrMeasChar.setFixedLen(8);
  // Optionally capture Client Characteristic Config Descriptor updates
  pwrMeasChar.setCccdWriteCallback(cccdCallback);
  pwrMeasChar.begin();

  /*
   * The other two characterstics aren't updated over time, they're static info
   * relaying what's available in our service and characteristics.
   */

  // Characteristic for power feature. Has to be readable, but not necessarily
  // notify. 32 bit value of what's supported, see
  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.cycling_power_feature.xml
  pwrFeatChar.setProperties(CHR_PROPS_READ);
  pwrFeatChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  // 1 32-bit value
  pwrFeatChar.setFixedLen(4);
  pwrFeatChar.begin();
  // set bit 3 because crank revolutions will be present
  // set bit 9 because offset calibration will be supported
  pwrFeatChar.write32(0x00000208);

  // Characteristic for sensor location. Has to be readable, but not necessarily
  // notify. See:
  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.sensor_location.xml
  pwrLocChar.setProperties(CHR_PROPS_READ);
  pwrLocChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pwrLocChar.setFixedLen(1);
  pwrLocChar.begin();
  // Set location to "left crank"
  pwrLocChar.write8(5);

  pwrCtlChar.setProperties(CHR_PROPS_INDICATE | CHR_PROPS_WRITE);
  pwrCtlChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  pwrCtlChar.setWriteCallback(writeCallback);
  pwrCtlChar.setCccdWriteCallback(cccdCallback);
  pwrCtlChar.begin();
}

/*
 * This service exists only to publish logs over BLE.
 */

void setupCFM() {
  cfmService.begin();
#ifdef ENABLE_BLE_LOG
  // Has nothing to do with any spec.
  logChar.setProperties(CHR_PROPS_NOTIFY);
  // First param is the read permission, second is write.
  logChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  // Payload is quite limited in BLE, so come up with good logging shorthand.
  logChar.setMaxLen(20);
  logChar.begin();
#endif
  // Has nothing to do with any spec.
  cfgChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  // First param is the read permission, second is write.
  cfgChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  // Payload is quite limited in BLE, so come up with good logging shorthand.
  cfgChar.setMaxLen(20);
  cfgChar.setWriteCallback(writeCallback);
  cfgChar.begin();
}



//#define SIMULATE
/*
 * Publish the instantaneous power measurement.
 */
void blePublishPower(int16_t instantPwr, uint16_t crankRevs, long millisLast) {
  // Power measure characteristic
  /**
   * Fields
   *
   * Flags (16 bits):
   *   b0 pedal power balance present
   *   b1 pedal power balance reference
   *   b2 accumulated torque present
   *   b3 accumulated torque source
   *   b4 wheel revolution data present
   *   b5 crank revolution data present
   *   b6 extreme force magnitudes present
   *   b7 extreme torque magnitudes present
   *   b8 extreme angles present
   *   b9 top dead spot angle present
   *   b10 bottom dead spot angle present
   *   b11 accumulated energy present
   *   b12 offset compenstation indicator
   *   b13 reserved
   *
   * Instananous Power:
   *   16 bits signed int
   *   
   * Cumulative Crank Revolutions:
   *   16 bits signed int
   *
   * Last Crank Event Time
   *   16 bits signed int
   */
  // Flag cadence. Put most-significant octet first, it'll flip later.
  uint16_t flag = 0b0000000000100000;
  //flag |= 0b0001000000000000;

  // All data in characteristics goes least-significant octet first.
  // Split them up into 8-bit ints. LSO ends up first in array.
  uint8_t flags[2];
  uint16ToLso(flag, flags);
  uint8_t pwr[2];
#ifdef SIMULATE
  instantPwr = 33;
#endif
  uint16ToLso(instantPwr, pwr);
  // Split the 16-bit ints into 8 bits, LSO is first in array.
  uint8_t cranks[2];
#ifdef SIMULATE
  static uint16_t crankRevsSim = 0;
  crankRevsSim += 4;
  crankRevs = crankRevsSim;
#endif
  uint16ToLso(crankRevs, cranks);
  uint8_t lastTime[2];

  // Cadance last event time is time of last event, in 1/1024 second resolution
  uint16_t lastEventTime = uint16_t(millisLast / 1000.f * 1024.f) % 65536;
  uint16ToLso(lastEventTime, lastTime);

  // All fields are 16-bit values, split into two 8-bit values.
  uint8_t pwrdata[8] = { flags[0], flags[1], pwr[0], pwr[1], cranks[0], cranks[1], lastTime[0], lastTime[1] };
  //uint8_t pwrdata[4] = { flags[0], flags[1], pwr[0], pwr[1] };

  //Log.notice("BLE published flags: %X %X pwr: %X %X cranks: %X %X last time: %X %X\n", 
  //           pwrdata[0], pwrdata[1], pwrdata[2], pwrdata[3], pwrdata[4], pwrdata[5], pwrdata[6], pwrdata[7]);

  if (pwrMeasChar.notify(pwrdata, sizeof(pwrdata))) {
#ifdef DEBUG
    //Serial.print(F("Updated Pwr/Cranks/Time to: ")); Serial.print(instantPwr); Serial.print(F("/")); Serial.print(crankRevs); Serial.print(F("/"));Serial.println(lastEventTime);
    //Serial.print(F("Power measurement updated to: "));
    //Serial.println(instantPwr);
  } else {
    Serial.println("ERROR: Power notify not set in the CCCD or not connected!");
#endif
  }
}

void blePublishTePs(float& te, float& ps) {
#ifdef ENABLE_EXT_SVCS
  notifyExtTePs(te, ps);
#endif
}

void blePublishBatt(uint8_t battPercent) {
  blebas.write(battPercent);
#ifdef DEBUG
  Serial.printf("Updated battery percentage to %d\n", battPercent);
#endif
}

/*
 * Publish a tiny little log message over BLE. Pass a null-terminated
 * char*, in 20 chars or less (counting the null).
 */
#ifdef ENABLE_BLE_LOG
void blePublishLog(const char* fmt, ...) {
  static const short MAX = 20;  // 19 chars plus the null terminator
  static char msg[MAX];

  va_list args;
  va_start(args, fmt);
  int numBytes = vsprintf(msg, fmt, args);
  va_end(args);

  if (numBytes < 0) {
    Serial.println("Failed to write BLE log to buffer");
  } else if (numBytes > MAX) {
    Serial.printf("Too many bytes written (%d), overflowed the msg buffer.\n", numBytes);
    Serial.printf("Original message: %s\n", msg);
  } else if (logChar.notifyEnabled()) {
    bool ret = logChar.notify(msg, numBytes);
    if (ret) {
      Serial.printf("Sent log %d byte message: %s\n", ret, msg);
    } else {
      Serial.println("Failed to publish log message over BLE.");
    }
  }
}
#endif

void bleSetup() {
  Bluefruit.begin();
  Bluefruit.setName(DEV_NAME);

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  // off Blue LED for lowest power consumption
  Bluefruit.autoConnLed(false);

  // Configure and Start the Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Start the BLE Battery Service
  blebas.begin();

  // Setup the Heart Rate Monitor service using
  // BLEService and BLECharacteristic classes
  setupPwr();

  setupCFM();

#ifdef ENABLE_EXT_SVCS
  setupExtSvcs();
#endif

  // Setup the advertising packet(s)
  startAdv();

#ifdef DEBUG
  Serial.println("BLE module configured and advertising.");
#endif
}