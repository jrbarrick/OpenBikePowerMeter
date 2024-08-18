#ifndef _BLE_SVCS_H_
#define _BLE_SVCS_H_

#include <set>

#define ENABLE_BLE_LOG

#define MSO(o) uint8_t(uint16_t(o) >> 8 & 0xff)
#define LSO(o) uint8_t(uint16_t(o) & 0xff)
#define LMOCS(o) LSO(o), MSO(o)

#define LOC_LEFT_CRANK 5
#define LOC_RIGHT_CRANK 6
#define FTR_P_PWR_BAL 0x0001
#define FTR_CRANK_REV 0x0008
#define FTR_OFS_CALIB 0x0200
#define FTR_MLTP_SENS 0x0800

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

typedef enum {
  REQ_INVALID = 0,
  REQ_CALIB_AUTO_L,
  REQ_CALIB_AUTO_R,
  REQ_CALIB_SET_L,
  REQ_CALIB_SET_R,
  REQ_CALIB_GET,
  REQ_CALIB_APPLY,
  REQ_CALIB_VERIFY,
  REQ_CALIB_PERSIST,
  REQ_SET_ED,
  REQ_GET_ED,
  REQ_SET_PAR,
  REQ_GET_PAR,
  REQ_SET_IPM,
  REQ_GET_IPM,
  REQ_LOG_MODE
} cfmReq_t;

typedef int16_t (bleOffsetCompCb_t)(bool& resultValid);
typedef int16_t (bleCfmReqCb_t)(cfmReq_t req, uint8_t* respBuf, float arg1, float arg2, float arg3);

void bleSetup();

void blePublishRevUpdate(revUpdt_t& revUpdate);

void blePublishBatSOC(uint8_t batSOC);
void bleSetOffsetCompensationCb(bleOffsetCompCb_t* bleOffsetCompCb);
void bleSetCfgAndMonRequestCb(bleCfmReqCb_t* bleCfmReqCb);

#ifdef ENABLE_BLE_LOG
void blePublishLog(const char* fmt, ...);
#endif


#endif //_BLE_SVCS_H_
