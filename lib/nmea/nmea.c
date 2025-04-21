#include "nmea.h"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

bool nmea_is_gngga_message(const char *message)
{
    if (message == NULL)
    {
        return false;
    }

    if (strncmp(message, "$GNGGA", 6) == 0)
    {
        return true;
    }

    return false;
}

bool nmea_gga_has_location_data(const char *gga_message)
{
    if (gga_message == NULL)
    {
        return false;
    }

    char message_copy[256];
    strncpy(message_copy, gga_message, sizeof(message_copy) - 1);
    message_copy[sizeof(message_copy) - 1] = '\0';

    char *p = strtok(message_copy, ",");
    int field = 1;

    bool has_lat = false;
    bool has_lon = false;
    int quality = -1;

    while (p != NULL)
    {
        switch (field)
        {
        case GGA_FIELD_LAT:
            if (strlen(p) > 0)
            {
                has_lat = true;
            }
            break;

        case GGA_FIELD_LON:
            if (strlen(p) > 0)
            {
                has_lon = true;
            }
            break;

        case GGA_FIELD_QUALITY:
            quality = atoi(p);
            break;
        }

        p = strtok(NULL, ",");
        field++;
    }

    return has_lat && has_lon && quality >= GPS_QUALITY_SPS && quality <= GPS_QUALITY_DEAD_RECK;
}

int nmea_get_gga_fix_quality(const char *gga_message)
{
    if (gga_message == NULL)
    {
        return -1;
    }

    char message_copy[256];
    strncpy(message_copy, gga_message, sizeof(message_copy) - 1);
    message_copy[sizeof(message_copy) - 1] = '\0';

    char *p = strtok(message_copy, ",");
    int field = 1;

    while (p != NULL && field <= GGA_FIELD_QUALITY)
    {
        if (field == GGA_FIELD_QUALITY)
        {
            return atoi(p);
        }
        p = strtok(NULL, ",");
        field++;
    }

    return -1;
}