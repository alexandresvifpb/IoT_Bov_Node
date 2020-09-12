#ifndef ND_LoRa_Lib_V01_h
#define ND_LoRa_Lib_V01_h

#include <ArduinoJson.h>
#include "LoRa.h"
#include "LinkedList.h"
#include "TimeLib.h"

#define LORA_PIN_SCK                5                 // GPIO5  -- SX1278's SCK
#define LORA_PIN_MISO               19                // GPIO19 -- SX1278's MISO
#define LORA_PIN_MOSI               27                // GPIO27 -- SX1278's MOSI
#define LORA_PIN_SS                 18                // GPIO18 -- SX1278's CS
#define LORA_PIN_RST                14                // GPIO14 -- SX1278's RESET
#define LORA_PIN_DIO0               26                // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define LORA_BAND                   915E6             // 433E6  //you can set band here directly,e.g. 868E6,915E6
#define LORA_SYN_WORD               0x00

#define TIMEOUT                     10000
#define BROADCAST                   "FFFFFF"
#define COUNTER_RECONNECTION        10                // reconnection time

#define LORA_TASK_DELAY_MS          10

typedef struct {
  String sourcer_device_id;
  String target_device_id;
  String message_id;
  uint8_t message_type;
  String neighbor_id_list;
  uint8_t hops;
  String payload;
} lora_send_t;

typedef struct {
  String sourcer_device_id;
  String target_device_id;
  String message_id;
  uint8_t message_type;
  String neighbor_id_list;
  uint8_t hops;
  float SNR;
  float RSSI;
  String payload;
} lora_recv_t;

typedef struct {
  float SNR;
  int16_t RSSI;
  String payload;
} lora_temp_t;

// #ifdef __cplusplus
// extern "C" {
// #endif

class LoRaNodeLib
{
  public:
    LoRaNodeLib();

    boolean begin(String id);
    void run(void);
    void add_send_message(lora_send_t msg);
    void add_recv_message(lora_recv_t msg);
    String get_next_send_message(void);
    lora_recv_t get_next_recv_message(void);
    boolean is_send_list_empty(void);
    boolean is_recv_list_empty(void);
    boolean is_connected(void);
    boolean send_message(String message);
    void set_connected(bool status);

  private:
    static void onReceive(int packetSize);
    void SetRxMode(void);
    void SetTxMode(void);
    String encodeJSON_recv(lora_recv_t msg);
    String encodeJSON_send(lora_send_t msg);
    static lora_recv_t decodeJSON_recv(String strJSON);
    static lora_send_t decodeJSON_send(String strJSON);

    uint8_t synWord = LORA_SYN_WORD;

};

// #ifdef __cplusplus
// }
// #endif

#endif  // ND_LoRa_Lib_V01_h