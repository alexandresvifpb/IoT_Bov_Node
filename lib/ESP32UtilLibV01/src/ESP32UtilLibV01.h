#ifndef ESP32UTILLIBV01_H 
#define ESP32UTILLIBV01_H

#include "ArduinoJson.h"
#include "LinkedList.h"
#include "TimeLib.h"
#include "EEPROM.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <mbedtls/md.h>

#define GATEWAY                           0
#define NODE                              1

#define BEACON                            0
#define JOIN                              1
#define JOIN_ACK                          2
#define REQUEST                           3
#define REPLAY                            4

#define RESET_CONT_BOOT                   false         // resets the bootsequence counter variable in the EEPROM
#define VBAT_PIN                          36
#define MINIMUM_BATTERY_VOLTAGE           7.00          // minimum battery voltage
#define COUNTER_ENTER_MODE_DEEPSLIEEP     20      // counterToEnterModeDeepSleep

#define IA_TASK_DELAY_MS                  1
#define MOVING_AVERAGE_ARRAY_SIZE         10

typedef struct {
  double gps_speed;
  uint32_t gps_age;
  double gps_distanceBetweenTwoPoints;
  float imu_Pitch;
} behavior_t;

// #ifdef __cplusplus
// extern "C" {
// #endif

class ESP32UtilLib
{
  public:
    ESP32UtilLib();

    String get_MAC(void);
    uint8_t hash(String str);
    uint16_t get_boot_sequence(void);
    float getVBat(void);
    boolean checkBattery(void);

    float moving_average_yaw(float value);
    float moving_average_pitch(float value);
    float moving_average_roll(float value);

    uint8_t classify_behavior(void);

  private:
    uint16_t getFlashUInt16(uint8_t address);
    void setFlashUInt16(uint8_t address, uint16_t value);

};

// #ifdef __cplusplus
// }
// #endif

#endif  // ESP32UTILLIBV01_H