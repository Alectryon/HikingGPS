#ifndef _GPSDATA_H
#define _GPSDATA_H

#include <stdint.h>

#define GPS_NOFIX 0
#define GPS_GPSFIX 1
#define GPS_DIFF 2

#define NMEA_NONE 0
#define NMEA_GPGGA 1

static const char CMD_OUTPUT[] = "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2C\r\n";
static const char CMD_STANDBY[]  = "$PMTK161,0*28\r\n";

static const char CMD_RATE1[] = "$PMTK200,1000*1F\r\n";
static const char CMD_RATE2[] = "$PMTK200,2000*1F\r\n";
static const char CMD_RATE3[] = "$PMTK200,3000*1F\r\n";
static const char CMD_RATE4[] = "$PMTK200,4000*1F\r\n";
static const char CMD_RATE5[] = "$PMTK200,5000*1F\r\n";
static const char CMD_RATE6[] = "$PMTK200,6000*1F\r\n";
static const char CMD_RATE7[] = "$PMTK200,7000*1F\r\n";
static const char CMD_RATE8[] = "$PMTK200,8000*1F\r\n";
static const char CMD_RATE9[] = "$PMTK200,9000*1F\r\n";
static const char CMD_RATE10[] = "$PMTK200,10000*1F\r\n";
/*const char* CMD_RATES[] =
{
	CMD_RATE1,
	CMD_RATE2,
	CMD_RATE3,
	CMD_RATE4,
	CMD_RATE5,
	CMD_RATE6,
	CMD_RATE7,
	CMD_RATE8,
	CMD_RATE9,
	CMD_RATE10
};*/
static char gpscmdstr[56];

static char GPSBuffer[128];
static int GPSBufferIndex = 0;

/**
	GPS support functions for NMEA
*/
// Data structure to hold the important GPS data
struct GPSData {
    uint8_t status;	// 0=no fix, 1=GPS fix, 2=Diff
    uint8_t numsats; // Number of satellites
    int time;	// hhmmss.ss (without the decimal)
    int altitude; // Altitude above sea-level
    float HDOP;	// Horizontal dilution of precision
    float longitude; // llll.ll (without the decimal, sign represents N or S)
    float latitude; // yyyy.yy
};

// Load the GPS data from UART
int parseGPSData(struct GPSData *gpsdata, char *GPSNMEA);

// Set the output statement frequencies
void setOutputs();

// Set the update rate (depends on baudrate)
void setUpdateRate(int16_t ms);

// Set GPS into standby mode for power saving
void enterStandby();

// Wake up by sending any byte
void wakeup();

#endif
