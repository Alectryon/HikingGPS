#include "GPSData.h"
#include <stdlib.h>
#include <string.h>

static char *GPS_token;
static char *GPS_deg;

int parseGPSData(struct GPSData *gpsdata, char *GPSNMEA) {
    GPS_token = strtok(GPSNMEA, ",");
    if (!strcmp(GPS_token, "$GPGGA")) {
        // UTC of position (time)
        GPS_token = strtok(NULL, ".");
        gpsdata->time = atoi(GPS_token);
        GPS_token = strtok(NULL, ",");

        // Latitude of position
        GPS_token = strtok(NULL, ",");
        //gpsdata.latStr = GPS_token;
        // First take the integer value divide 100
        gpsdata->latitude = atoi(GPS_token)/100;
        GPS_deg = strrchr(GPS_token, '.');
        GPS_deg -= 2;
        gpsdata->latitude += atof(GPS_deg)/60;

        // North or South (South is negative)
        GPS_token = strtok(NULL, ",");
        if (!strcmp(GPS_token, "S"))
            gpsdata->latitude *= -1;

        // Longitude of position
        GPS_token = strtok(NULL, ",");
        //gpsdata.longStr = GPS_token;
        gpsdata->longitude = atoi(GPS_token)/100;
        GPS_deg = strrchr(GPS_token, '.');
        GPS_deg -= 2;
        gpsdata->longitude += atof(GPS_deg)/60;

        // East or West (West is negative)
        GPS_token = strtok(NULL, ",");
        if (!strcmp(GPS_token, "W"))
            gpsdata->longitude *= -1;

        // GPS quality/status
        GPS_token = strtok(NULL, ",");
        gpsdata->status = atoi(GPS_token);

        // # of Satellites
        GPS_token = strtok(NULL, ",");
        gpsdata->numsats = atoi(GPS_token);

        // Horizontal dilution of precision
        GPS_token = strtok(NULL, ",");

        // Antenna altitude above mean sea level
        GPS_token = strtok(NULL, ",");
        gpsdata->altitude = atoi(GPS_token);

        // Altitude units (meters)
        GPS_token = strtok(NULL, ",");

        // new data, so return 1
        return 1;
    }
    // TODO: Include $GPRMC
    return 0;
}

// Set the output statement frequencies
// Sets so only NMEA GGA statements are 1 per fix
void setOutputs()
{
    /*strcpy_P(gpscmdstr, CMD_OUTPUT);
    for (uint8_t i = 0; i < strlen(gpscmdstr); ++i) {
        UART::writeByte(gpscmdstr[i]);
    }*/
}

// Set the update rate (depends on baudrate)
void setUpdateRate(int16_t ms)
{
    // Switch statement easier
    /*strcpy_P(gpscmdstr, (PGM_P)pgm_read_word(&(CMD_RATES[ms/1000])));
    for (uint8_t i = 0; i < strlen(gpscmdstr); ++i) {
        UART::writeByte(gpscmdstr[i]);
    }*/
}

// Set GPS into standby mode for power saving
void enterStandby()
{
    /*strcpy_P(gpscmdstr, CMD_STANDBY);
    for (uint8_t i = 0; i < strlen(gpscmdstr); ++i) {
        UART::writeByte(gpscmdstr[i]);
    }*/
}

// Wake up by sending any byte
void wakeup()
{
    //UART::writeByte(' ');
}
