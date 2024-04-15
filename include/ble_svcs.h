#ifndef _BLE_SVCS_H_
#define _BLE_SVCS_H_

#define ENABLE_BLE_LOG

#define MSO(o) uint8_t(uint16_t(o) >> 8 & 0xff)
#define LSO(o) uint8_t(uint16_t(o) & 0xff)
#define LMOCS(o) LSO(o), MSO(o)

typedef struct {
  long  time = 0;
  float powerL = 0;
  float powerR = 0;
  float teL = 0;
  float teR = 0;
  float psL = 0;
  float psR = 0;
  uint16_t crankRev = 0;
} revUpdt_t;

typedef int16_t (bleOffsetCompCb_t)(bool& resultValid);

void bleSetup();

void blePublishRevUpdate(revUpdt_t& revUpdate);

void blePublishBatSOC(uint8_t batSOC);
void bleSetOffsetCompensationCb(bleOffsetCompCb_t* bleOffsetCompCb);

#ifdef ENABLE_BLE_LOG
void blePublishLog(const char* fmt, ...);
#endif


#endif //_BLE_SVCS_H_
