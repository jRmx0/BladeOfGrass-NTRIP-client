#ifndef RTK_H
#define RTK_H

#include <stdbool.h>

// GGA message field indices
#define GGA_FIELD_HEADER        1
#define GGA_FIELD_UTC           2
#define GGA_FIELD_LAT           3
#define GGA_FIELD_LAT_DIR       4
#define GGA_FIELD_LON           5
#define GGA_FIELD_LON_DIR       6
#define GGA_FIELD_QUALITY       7
#define GGA_FIELD_SATELLITES    8
#define GGA_FIELD_HDOP          9
#define GGA_FIELD_ALTITUDE      10
#define GGA_FIELD_ALT_UNITS     11
#define GGA_FIELD_GEOID_SEP     12
#define GGA_FIELD_GEOID_UNITS   13

// GPS Quality Indicator values
#define GPS_QUALITY_INVALID      0  
#define GPS_QUALITY_SPS          1  
#define GPS_QUALITY_DIFFERENTIAL 2 
#define GPS_QUALITY_PPS          3
#define GPS_QUALITY_RTK_INT      4
#define GPS_QUALITY_RTK_FLOAT    5
#define GPS_QUALITY_DEAD_RECK    6 
#define GPS_QUALITY_MANUAL       7
#define GPS_QUALITY_SIMULATOR    8 

/**
 * @brief Check if a message is a valid GNGGA message
 * 
 * @param message The NMEA message to check
 * @return true if the message starts with $GNGGA
 * @return false otherwise
 */
bool nmea_is_gngga_message(const char *message);

/**
 * @brief Check if a GGA message has valid location data
 * 
 * Message is considered valid if:
 * 1. It has latitude and longitude data
 * 2. Quality indicator is between 1 and 6 inclusive
 *    (GPS_QUALITY_SPS, GPS_QUALITY_DIFFERENTIAL, GPS_QUALITY_PPS, 
 *     GPS_QUALITY_RTK_INT, GPS_QUALITY_RTK_FLOAT, GPS_QUALITY_DEAD_RECK)
 * 
 * @param gga_message The GGA NMEA message to check
 * @return true if the message contains valid location data
 * @return false if the message is NULL or doesn't contain valid location data
 */
bool nmea_gga_has_location_data(const char *gga_message);

/**
 * @brief Get the GPS fix quality from a GGA message
 * 
 * @param gga_message The GGA NMEA message to parse
 * @return int The fix quality value (0-8) or -1 if not found
 */
int nmea_get_gga_fix_quality(const char *gga_message);

#endif /* RTK_H */