#ifndef	_COMMON_H
#define	_COMMON_H

#include <SLFS.h>
#include <WiFi.h>

// IO definitions
// 30 is GPIO 0 / 31 is GPIO 24 / 32 is GPIO 23
#define POWER_LED          4
#define SWITCH_INPUT       30  
#define BATT_MONITORING    33
#define PWM_PIN            31
#define OTHER_IO           32

// Analog Inputs are GPIO 3 (2) and GPIO 4 (6)
#define ANALOG_INPUT1      2
#define ANALOG_INPUT2      6

// DEFAULTS
#define DEFAULT_UDP_PORT  8888
#define DEFAULT_UDP_SERVICE_PORT  9999
#define DEFAULT_SAMPLE_RATE  5
#define VERSION_DATE        "R-IoT v1.5 - IRCAM 2015"
#define PARAMS_FILENAME     "params.txt"
#define WEB_SERVER_DELAY    100          // Time to press on the switch to start the webserver      

#define GYRO_NOISEGATE      50

// Sensor orientation
#define TOP      0
#define BOTTOM   1

// Declination at Paris, FRANCE (about 1 minute)
#define DECLINATION    0.01
//#define DECLINATION  13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04

#define MAX_CLIENTS    5
#define STATION_MODE   0
#define AP_MODE        1

typedef union uWord {
	short int Value;
  	unsigned char Val[sizeof(short int)];
} Word;

// For Mag calibration stages
typedef enum coords {X, Y, Z};
typedef enum CalStage {START, X_PLUS_0, X_PLUS_180, X_MINUS_0, X_MINUS_180, Y_PLUS_0, Y_PLUS_180, Y_MINUS_0, Y_MINUS_180, Z_PLUS_0, Z_PLUS_180, Z_MINUS_0, Z_MINUS_180, END}; 


#define IPV4_SIZE  4
#define MAX_SERIAL 80
#define MAX_STRING_LEN  80
#define MAX_STRING      200

// Serial Messages
#define TEXT_WIFI_MODE    "mode"
#define TEXT_SSID	  "ssid"
#define TEXT_OWNIP	  "ownip"
#define TEXT_DESTIP	  "destip"
#define TEXT_GATEWAY	  "gateway"
#define TEXT_DNS	  "dns"
#define TEXT_MASK	  "mask"
#define TEXT_PORT	  "port"
#define TEXT_MASTER_ID	  "masterid"
#define TEXT_SAMPLE_RATE  "samplerate"
#define TEXT_SECURITY     "security"
#define TEXT_PASSWORD     "pass"
#define TEXT_DHCP         "dhcp"

// Offsets & calibration matrix
#define TEXT_ACC_OFFSETS  "acc_offset"
#define TEXT_GYRO_OFFSETS  "gyr_offset"
#define TEXT_MAG_OFFSETS  "mag_offset"

#define TEXT_MATRIX        "matrix"

#define MIN_SAMPLE_RATE     3
#define MAX_SAMPLE_RATE	    1000


void ParseIP(char *TheString, IPAddress *TheIP);
unsigned char SkipToValue(char *StringBuffer);
unsigned char SkipToNextValue(char *StringBuffer, unsigned char StartIndex);
unsigned char GrabLine(char *StringBuffer);
unsigned int StringLength(char* StringBuffer);
char * ftoa(double f, char * buf, int precision);



#endif
