#ifndef ND_GPS_Lib_V01_h
#define ND_GPS_Lib_V01_h

#include <TinyGPS++.h>
#include <Time.h>
#include "TimeLib.h"

#define GPS_PIN_RX          12
#define GPS_PIN_TX          15
#define GPS_BAUD            9600
#define GPS_UTC_OFFSET      -3

#define GPS_LAST_TIME       5000
#define GPS_TASK_DELAY_MS   1000

typedef struct {
  String device_id;
  unsigned long epochTime;
  double latitude;
  double longitude;
  double altitude;
  double speed;
  double course;
  double distanceBetweenTwoPoints;
  uint32_t age;
  uint32_t satellites;
  int32_t hdop;
} data_gps_t;

// #ifdef __cplusplus
// extern "C" {
// #endif

class GPSLib
{
  public:
    GPSLib();

    boolean begin(String id);
    void run(void);
    boolean new_record(void);
    data_gps_t get_record(void);
    String get_string_record(data_gps_t data_gps, long nowtime);
    unsigned long epochtime(void);

  private:

};

// #ifdef __cplusplus
// }
// #endif

#endif  // ND_GPS_Lib_V01_h