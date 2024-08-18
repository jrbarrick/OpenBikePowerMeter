#include <Arduino.h>
#include "cfg.h"

InternalFileSystem Ifs;

char CfgDir[] = "/cfg";
char CfgClbPath[] = "/cfg/CLB.bin";
char CfgRtPath[] = "/cfg/RT.bin";

void cfgSetup() {
  Ifs.begin();
  if(!Ifs.exists(CfgDir)) {
    Ifs.mkdir(CfgDir);
  }
}

void cfgClose() {
  Ifs.end();
}

void cfgWrite(CfgRuntime& cfgRuntime) {
  Ifs.begin();
  Adafruit_LittleFS_Namespace::File cfgRtFile = Ifs.open(CfgRtPath, Adafruit_LittleFS_Namespace::FILE_O_WRITE);
  if (cfgRtFile.isOpen()) {
    cfgRtFile.write((uint8_t*)&cfgRuntime, sizeof(cfgRuntime));
    cfgRtFile.close();
  }
}

void cfgWrite(CfgCalib& cfgCalib) {
  Ifs.begin();
  Adafruit_LittleFS_Namespace::File cfgClbFile = Ifs.open(CfgClbPath, Adafruit_LittleFS_Namespace::FILE_O_WRITE);
  if (cfgClbFile.isOpen()) {
    cfgClbFile.write((uint8_t*)&cfgCalib, sizeof(cfgCalib));
    cfgClbFile.close();
  }
}

void cfgRead(CfgCalib& cfgCalib) {
  Ifs.begin();
  if (Ifs.exists(CfgClbPath)) {
    Adafruit_LittleFS_Namespace::File cfgClbFile = Ifs.open(CfgClbPath, Adafruit_LittleFS_Namespace::FILE_O_READ);
    if (cfgClbFile.isOpen()) {
        cfgClbFile.read((uint8_t*)&cfgCalib, sizeof(cfgCalib));
        cfgClbFile.close();
    }
  }
}

void cfgRead(CfgRuntime& cfgRuntime) {
  Ifs.begin();
  if (Ifs.exists(CfgRtPath)) {
    Adafruit_LittleFS_Namespace::File cfgRtFile = Ifs.open(CfgRtPath, Adafruit_LittleFS_Namespace::FILE_O_READ);
    if (cfgRtFile.isOpen()) {
        cfgRtFile.read((uint8_t*)&cfgRuntime, sizeof(cfgRuntime));
        cfgRtFile.close();
    }
  }
}

void cfgRead(CfgCalib& cfgCalib, CfgRuntime& cfgRuntime) {
  cfgRead(cfgCalib);
  cfgRead(cfgRuntime);
}

void cfgSync(CfgRuntime& cfgRuntime) {
    CfgRuntime cfgRtOld;
    cfgRead(cfgRtOld);
    if(memcmp(&cfgRuntime, &cfgRtOld, sizeof(CfgRuntime))) {
      cfgWrite(cfgRuntime);
      Serial.println("Cfg synced.");
    }
}
