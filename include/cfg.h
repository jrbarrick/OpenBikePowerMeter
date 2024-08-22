#include <InternalFileSystem.h>


#define CFG_CLB_VER 1
#define CFG_RT_VER 1


typedef struct {
  uint32_t cfgVer;
  float lcForceCalibFctL;
  float lcForceCalibFctR;
} CfgCalib;

typedef struct {
  uint32_t cfgVer;
  uint32_t crankRadius;
  uint32_t pwrAvgRevs;
  float pwrExpDecFctr;
  bool instPwrMeas;
} CfgRuntime;

void cfgSetup();
void cfgClose();
void cfgSync(CfgRuntime& cfgRuntime);
void cfgSync(CfgCalib& cfgCalib);
void cfgWrite(CfgRuntime& cfgRuntime);
void cfgWrite(CfgCalib& cfgCalib);
void cfgRead(CfgCalib& cfgCalib);
void cfgRead(CfgRuntime& cfgRuntime);
void cfgRead(CfgCalib& cfgCalib, CfgRuntime& cfgRuntime);
