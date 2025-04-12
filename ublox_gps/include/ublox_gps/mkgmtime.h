

#ifndef UBLOX_GPS_MKGMTIME_H
#define UBLOX_GPS_MKGMTIME_H

#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get the UTC time in seconds and nano-seconds from a time struct in
 * GM time.
 */
time_t mkgmtime(struct tm * const tmp);

#ifdef __cplusplus
}
#endif

#endif // UBLOX_GPS_MKGMTIME_H
