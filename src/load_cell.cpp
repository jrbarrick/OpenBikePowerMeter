/**
 * Force and load cell-specific code and helpers. HX711 chip.
 */

#include <Arduino.h>
#include <climits>
#include "load_cell.h"

//#define DEBUG

#ifdef _VARIANT_FEATHER52840_SENSE_
#define PIN_LOAD_SCK  5
#define PIN_LOAD_D0_L 6
#define PIN_LOAD_D0_R 9
#else
#define PIN_LOAD_SCK  29
#define PIN_LOAD_D0_L 28
#define PIN_LOAD_D0_R 24
#endif

//#define ENABLE_RIGHT_LOAD_CELL

//#define USE_ONLY_16BIT
#define MASK_RESULT_VALUE_PATTERN 0xffffff00

/*
 * 1: => channel A, gain factor 128 (hx711 default)
 * 2: => channel B, gain factor 32
 * 3: => channel A, gain factor 64
*/
#define DEFAULT_SRC_GAIN_OPT 1

long LOAD_OFFSET_CHA_L = 0;
long LOAD_OFFSET_CHA_R = 0;

float LOAD_SCALE_FCT_CHA_L = 1;
float LOAD_SCALE_FCT_CHA_R = 1;

bool lcAvailableL = false;
bool lcAvailableR = false;

lcUpdateIsrCb_t* loadUpdateCb = nullptr;

uint32_t READ_TIMEOUT_CTR_INIT_VALUE = (1 / (float)LOAD_CELL_EXP_SPS_RATE * 1700000);


// See also:
// - https://github.com/bogde/HX711/issues/75
// - https://github.com/arduino/Arduino/issues/6561
// - https://community.hiveeyes.org/t/using-bogdans-canonical-hx711-library-on-the-esp32/539

//#define USE_FREE_RTOS_CRITICAL
#ifdef USE_FREE_RTOS_CRITICAL
#define FREE_RTOS_ENTER_CRITICAL taskENTER_CRITICAL();portENTER_CRITICAL();portDISABLE_INTERRUPTS();
#define FREE_RTOS_LEAVE_CRITICAL taskEXIT_CRITICAL();portEXIT_CRITICAL();portENABLE_INTERRUPTS();
#else
#define FREE_RTOS_ENTER_CRITICAL
#define FREE_RTOS_LEAVE_CRITICAL
#endif
#define ENTER_CRITICAL_SECTION FREE_RTOS_ENTER_CRITICAL noInterrupts();
#define LEAVE_CRITICAL_SECTION FREE_RTOS_LEAVE_CRITICAL interrupts();

bool waitForReady() {
	uint32_t tOutCtr = READ_TIMEOUT_CTR_INIT_VALUE;
	while ((lcAvailableL && digitalRead(PIN_LOAD_D0_L)) || (lcAvailableR && digitalRead(PIN_LOAD_D0_R)))
	{
		if (tOutCtr-- == 0) {
			return false;
		}
	}
	return true;
}

inline void readInternal(int32_t& valueL, int32_t& valueR, uint32_t nextSrcGainOpt) {
	valueL = 0;
	valueR = 0;
	ENTER_CRITICAL_SECTION
	for (uint8_t i = 0; i < 24; i++) {
		digitalWrite(PIN_LOAD_SCK, HIGH);
		digitalWrite(PIN_LOAD_SCK, LOW);
		valueL = valueL << 1;
		valueL |= digitalRead(PIN_LOAD_D0_L);
		valueR = valueR << 1;
		valueR |= digitalRead(PIN_LOAD_D0_R);
	}
	for (uint32_t i = 0; i < nextSrcGainOpt; i++) {
		digitalWrite(PIN_LOAD_SCK, HIGH);
		digitalWrite(PIN_LOAD_SCK, LOW);
	}
	LEAVE_CRITICAL_SECTION

	if(valueL & 0x800000) {
		valueL |= 0xff000000;
	}
	if(valueR & 0x800000) {
		valueR |= 0xff000000;
	}

	#ifdef MASK_RESULT_VALUE_PATTERN
	valueL &= MASK_RESULT_VALUE_PATTERN;
	valueR &= MASK_RESULT_VALUE_PATTERN;
	#endif
	#ifdef USE_ONLY_16BIT
	valueL = valueL >> 8;
	valueR = valueR >> 8;
	#endif
}

bool read(int32_t& valueL, int32_t& valueR, uint32_t srcGainOpt = DEFAULT_SRC_GAIN_OPT, bool defaultSrcGainOpt = false) {
	static uint32_t activeSrcGainOpt = 0;
	if (srcGainOpt != activeSrcGainOpt) {
		int32_t dValueL, dValueR;
		if (!waitForReady())
			return false;

		readInternal(dValueL, dValueR, srcGainOpt);
		if (defaultSrcGainOpt)
			activeSrcGainOpt = srcGainOpt;
	}
	if (!waitForReady())
		return false;

	readInternal(valueL, valueR, activeSrcGainOpt);
	return true;
}

bool lcMeasureAndAvgWithMinMax(float& avgValueL, float& avgValueR, int& minValueL, int& minValueR, int& maxValueL, int& maxValueR, long numOfMeasurements) {
	avgValueL = avgValueR = 0;
	minValueL = minValueR = INT_MAX;
	maxValueL = maxValueR = INT_MIN;

	for (long i = 0; i < numOfMeasurements; i++) {
    	static int32_t valueL = 0;
    	static int32_t valueR = 0;
		if(!read(valueL, valueR))
			return false;

		valueL -= LOAD_OFFSET_CHA_L;
		valueR -= LOAD_OFFSET_CHA_R;

		if (valueL < minValueL) minValueL = valueL;
		if (valueR > minValueR) minValueR = valueR;
		if (valueL > maxValueL) maxValueL = valueL;
		if (valueR > maxValueR) maxValueR = valueR;

    	avgValueL += (float)valueL / numOfMeasurements;
    	avgValueR += (float)valueR / numOfMeasurements;
	}

	return true;
}

void lcMeasureAndAvgFiltered(float& avgValueL, float& avgValueR, long numOfMeasurements) {
	double sumValueL = 0, sumValueR = 0;
    int32_t maxValueL = INT_MIN;
    int32_t minValueL = INT_MAX;
    int32_t maxValueR = INT_MIN;
    int32_t minValueR = INT_MAX;
	// just filter most significant min. and max. outliers
	for (long i = 0; i < numOfMeasurements + 2; i++) {
    	static int32_t valueL = 0;
    	static int32_t valueR = 0;
		read(valueL, valueR);
    	sumValueL += valueL;
    	sumValueR += valueR;
    	if (valueL < minValueL) minValueL = valueL;
    	if (valueL > maxValueL) maxValueL = valueL;
    	if (valueR < minValueR) minValueR = valueR;
    	if (valueR > maxValueR) maxValueR = valueR;
	}
    sumValueL -= minValueL;
	sumValueL -= maxValueL;
    sumValueR -= minValueR;
	sumValueR -= maxValueR;
	
	avgValueL = sumValueL / numOfMeasurements;
	avgValueR = sumValueR / numOfMeasurements;
}

void measureAndAvgRaw(float& avgValueL, float& avgValueR, long numOfMeasurements) {
	avgValueL = avgValueR = 0;
	static int32_t valueL = 0;
	static int32_t valueR = 0;
	for (long i = 0; i < numOfMeasurements; i++) {
		read(valueL, valueR);
    	avgValueL += (float)valueL / numOfMeasurements;
    	avgValueR += (float)valueR / numOfMeasurements;
	}
}

void lcDoOffsetCalib(long numOfMeasurements) {
	float offsetL = 0;
	float offsetR = 0;
	measureAndAvgRaw(offsetL, offsetR, numOfMeasurements);
  	LOAD_OFFSET_CHA_L = offsetL;
    LOAD_OFFSET_CHA_R = offsetR;
}

inline void scaleValueChA(int32_t& valueL, int32_t&valueR, float& sValueL, float& sValueR) {
	valueL -= LOAD_OFFSET_CHA_L;
	valueR -= LOAD_OFFSET_CHA_R;

	sValueL = (float)valueL / LOAD_SCALE_FCT_CHA_L;
	sValueR = (float)valueR / LOAD_SCALE_FCT_CHA_R;

	if(!lcAvailableL) {
		sValueL = sValueR;
	} else if(!lcAvailableR) {
		sValueR = sValueL;
	}
}

bool lcGetValue(float& sValueL, float& sValueR) {
    int32_t valueL = 0;
    int32_t valueR = 0;
	if(!read(valueL, valueR))
		return false;

	scaleValueChA(valueL, valueR, sValueL, sValueR);
	return true;
}

void onDataReady() {
	if (!(lcAvailableL && digitalRead(PIN_LOAD_D0_L)) && !(lcAvailableR && digitalRead(PIN_LOAD_D0_R)))
	{
		if (loadUpdateCb) {
			float sValueL, sValueR;
			int32_t valueL, valueR;
			readInternal(valueL, valueR, DEFAULT_SRC_GAIN_OPT);
			scaleValueChA(valueL, valueR, sValueL, sValueR);
			loadUpdateCb(sValueL, sValueR, valueL, valueR);
		}
	}
}

void lcGetAvailableLCs(bool& availableL, bool& availableR) {
	availableL = lcAvailableL;
	availableR = lcAvailableR;
}

void lcSetup() {
	digitalWrite(PIN_LOAD_SCK, HIGH);
	pinMode(PIN_LOAD_SCK, OUTPUT_H0H1);
	pinMode(PIN_LOAD_D0_L, INPUT_PULLUP);
	pinMode(PIN_LOAD_D0_R, INPUT_PULLUP);

	lcSoftPowerUp();

	uint32_t tOutCtr = READ_TIMEOUT_CTR_INIT_VALUE;
	int32_t valueL, valueR;
	readInternal(valueL, valueR, 1);
	while (tOutCtr--) {
		if (!digitalRead(PIN_LOAD_D0_L)) {
			lcAvailableL = true;
		}
		if (!digitalRead(PIN_LOAD_D0_R)) {
			lcAvailableR = true;
		}
		if (lcAvailableL && lcAvailableR) {
			return;
		}
	}
}

void lcStartContValueUpdate(lcUpdateIsrCb_t* ldUpdateCb) {
    if (ldUpdateCb == nullptr || loadUpdateCb != nullptr) {
        return;
	}
    loadUpdateCb = ldUpdateCb;
	int32_t valueL, valueR;
	read(valueL, valueR);
	if(lcAvailableL) {
    	attachInterrupt(digitalPinToInterrupt(PIN_LOAD_D0_L), onDataReady, FALLING);
	}
	if(lcAvailableR) {
    	attachInterrupt(digitalPinToInterrupt(PIN_LOAD_D0_R), onDataReady, FALLING);
	}
}

void lcStopContValueUpdate() {
	loadUpdateCb = nullptr;
	if(lcAvailableL) {
    	detachInterrupt(digitalPinToInterrupt(PIN_LOAD_D0_L));
	}
	if(lcAvailableR) {
    	detachInterrupt(digitalPinToInterrupt(PIN_LOAD_D0_R));
	}
}

void lcSoftPowerUp() {
	digitalWrite(PIN_LOAD_SCK, LOW);
    delay(20); //TODO: add proper wait for first ready here since load cell needs some time at start
}

void lcSoftPowerDown() {
	lcStopContValueUpdate();
	digitalWrite(PIN_LOAD_SCK, LOW);
	digitalWrite(PIN_LOAD_SCK, HIGH);
}

void lcSetScaleFactor(float scaleFctL, float scaleFctR) {
	LOAD_SCALE_FCT_CHA_L = scaleFctL;
	LOAD_SCALE_FCT_CHA_R = scaleFctR;
}

void lcGetScaleFactor(float& scaleFctL, float& scaleFctR) {
	scaleFctL = LOAD_SCALE_FCT_CHA_L;
	scaleFctR = LOAD_SCALE_FCT_CHA_R;
}

