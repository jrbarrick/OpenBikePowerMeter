#ifndef _LOAD_CELL_H_
#define _LOAD_CELL_H_

#define LOAD_CELL_EXP_SPS_RATE 296.58565f

typedef void (lcUpdateIsrCb_t)(float& sValueL, float& sValueR, int32_t& valueL, int32_t& valueR);

void lcSetup();
void lcSoftPowerUp();
void lcSoftPowerDown();
void lcSetScaleFactor(float scaleFctL, float scaleFctR);
void lcGetScaleFactor(float& scaleFctL, float& scaleFctR);
void lcDoOffsetCalib(long numOfMeasurements = 30);
void lcMeasureAndAvgFiltered(float& avgValueL, float& avgValueR, long numOfMeasurements);
bool lcMeasureAndAvgWithMinMax(float& avgValueL, float& avgValueR, int& minValueL, int& minValueR, int& maxValueL, int& maxValueR, long numOfMeasurements);
void lcStartContValueUpdate(lcUpdateIsrCb_t* ldUpdateCb);
void lcStopContValueUpdate();
bool lcGetValue(float& sValueL, float& sValueR);
void lcGetAvailableLCs(bool& availableL, bool& availableR);

#endif //_LOAD_CELL_H_
