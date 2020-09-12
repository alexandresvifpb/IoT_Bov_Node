#ifndef ND_SDCard_Lib_V01_h
#define ND_SDCard_Lib_V01_h

#include "Arduino.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "LinkedList.h"

#define SD_PIN_SCK              14
#define SD_PIN_MISO             2
#define SD_PIN_MOSI             13
#define SD_PIN_SS               25

#define SD_TASK_DELAY_MS        1

typedef struct {
    String id;
    uint16_t bootSequence;
    uint8_t type;
    String payload;
} SDCard_record_t;

enum Tscale {
  TYPE_ALL = 0,
  TYPE_IMU,
  TYPE_GPS,
  TYPE_HRSPO2,
};

class SDCardLib
{
  public:
    SDCardLib();                // construtor

    bool begin(void);
    void run(void);
    bool is_SDCard_present(void);
    bool add_record(SDCard_record_t value);
    bool append_file(fs::FS &fs, const char * filename, const char * payload);

  private:

};

#endif  // ND_SDCard_Lib_V01_h