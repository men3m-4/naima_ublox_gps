
#include "ublox_gps/mkgmtime.h"

static int tmcomp(const struct tm * const  atmp,
                  const struct tm * const btmp)
{
  int result;

  if ((result = (atmp->tm_year - btmp->tm_year)) == 0 &&
      (result = (atmp->tm_mon - btmp->tm_mon)) == 0 &&
      (result = (atmp->tm_mday - btmp->tm_mday)) == 0 &&
      (result = (atmp->tm_hour - btmp->tm_hour)) == 0 &&
      (result = (atmp->tm_min - btmp->tm_min)) == 0) {
    result = atmp->tm_sec - btmp->tm_sec;
  }
  return result;
}

time_t mkgmtime(struct tm * const tmp)
{
  int            dir;
  int            bits;
  int            saved_seconds;
  time_t              t;
  struct tm           yourtm, *mytm;

  yourtm = *tmp;
  saved_seconds = yourtm.tm_sec;
  yourtm.tm_sec = 0;
  /*
   * Calculate the number of magnitude bits in a time_t
   * (this works regardless of whether time_t is
   * signed or unsigned, though lint complains if unsigned).
  */
  for (bits = 0, t = 1; t > 0; ++bits, t <<= 1)
  {
  }

  /*
   * If time_t is signed, then 0 is the median value,
   * if time_t is unsigned, then 1 << bits is median.
  */
  t = (t < 0) ? 0 : ((time_t) 1 << bits);

  /* Some gmtime() implementations are broken and will return
   * NULL for time_ts larger than 40 bits even on 64-bit platforms
   * so we'll just cap it at 40 bits */
  if (bits > 40) {
    bits = 40;
  }

  for ( ; ; ) {
    mytm = gmtime(&t);

    if (!mytm) {
      return -1;
    }

    dir = tmcomp(mytm, &yourtm);
    if (dir != 0) {
      if (bits-- < 0) {
        return -1;
      }
      if (bits < 0) {
        --t;
      }
      else if (dir > 0) {
        t -= (time_t) 1 << bits;
      }
      else {
        t += (time_t) 1 << bits;
      }
      continue;
    }
    break;
  }
  t += saved_seconds;
  return t;
}
