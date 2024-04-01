#ifndef _BLE_SVCS_H_
#define _BLE_SVCS_H_

#define ENABLE_BLE_LOG

typedef int16_t (bleOffsetCompCb_t)(bool& resultValid);

void bleSetup();

void blePublishPower(int16_t instantPwr, uint16_t crankRevs, long millisLast);

void blePublishBatt(uint8_t battPercent);
void bleSetOffsetCompensationCb(bleOffsetCompCb_t* bleOffsetCompCb);

void blePublishTePs(float& te, float& ps);

#ifdef ENABLE_BLE_LOG
void blePublishLog(const char* fmt, ...);
#endif


#endif //_BLE_SVCS_H_
