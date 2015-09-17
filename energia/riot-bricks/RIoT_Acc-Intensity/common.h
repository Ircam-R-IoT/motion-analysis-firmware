#ifndef	_COMMON_H
#define	_COMMON_H

#include <WiFi.h>

// IO definitions
// 30 is GPIO 0 / 31 is GPIO 24 / 32 is GPIO 23
#define POWER_LED          4
#define SWITCH_INPUT       30  
#define BATT_MONITORING    33

// DEFAULTS
#define GYRO_NOISEGATE      50

// Sensor orientation
#define TOP      0
#define BOTTOM   1

// Declination at Paris, FRANCE (about 1 minute)
#define DECLINATION    0.01
//#define DECLINATION  13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04


typedef union uWord {
	short int Value;
  	unsigned char Val[sizeof(short int)];
} Word;

// For Mag calibration stages
typedef enum coords {X, Y, Z};
typedef enum CalStage {START, X_PLUS_0, X_PLUS_180, X_MINUS_0, X_MINUS_180, Y_PLUS_0, Y_PLUS_180, Y_MINUS_0, Y_MINUS_180, Z_PLUS_0, Z_PLUS_180, Z_MINUS_0, Z_MINUS_180, END}; 

#define MAX_STRING_LEN  80
#define MAX_STRING      200

#endif
