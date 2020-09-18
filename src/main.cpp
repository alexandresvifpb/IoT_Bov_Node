#include <Arduino.h>
#include "esp_task.h"
#include "esp_task_wdt.h"
                                        // https://github.com/alexandresvifpb/IoT_Bov_Node_Ver-01.git
                                        // D:\Users\alexa\OneDrive\doutorado\2020\prototipos\firmware\node\Node_LoRa_GPS_IMU_IA_TTGO-TBeam_V01          // localizacao do projeto
#define MAIN_MESSAGE_INITIAL            ("D:\\Users\\alexa\\OneDrive\\doutorado\\2020\\prototipos\\firmware\\tests\\Node_LoRa_GPS_IMU_IA_TTGO-TBeam_V01.02")
#define MAIN_DEBUG                      (true)          // Variable that enables (1) or disables (0) the sending of data by serial to debug the program
#define MAIN_CORE_0                     (0)             // Defined the core to be used
#define MAIN_CORE_1                     (1)             
#define MAIN_WATCHDOC_TIME_OUT          (600)           // Watchdog wait time to restart the process in seconds

TaskHandle_t TaskIdLoRa;
TaskHandle_t TaskIdSDCard;
TaskHandle_t TaskIdGPS;
TaskHandle_t TaskIdIMU;

void TaskLoRa( void * pvParameters );
void TaskSDCard( void * pvParameters );
void TaskGPS( void * pvParameters );
void TaskIMU( void * pvParameters );

//===========================================
//  ESP32 Util
#include "ESP32UtilLibV01.h"

ESP32UtilLib module;
String device_id = module.get_MAC();
String gateway_id = "FFFFFF";
uint8_t device_type = NODE;
uint16_t boot_sequence = -1;
behavior_t behavior_paramenters;
uint8_t behavior = 0;

//===========================================
//  LoRa SX1276/8
#include "ND_LoRa_Lib_V01.h"

LoRaNodeLib lora;
boolean lora_enable = false;

//===========================================
//  SD Card
#include "ND_SDCard_Lib_V01.h"

SDCardLib sdCard;
bool sdCard_enable = false;

//===========================================
//  GPS NEO6MV2
#include "ND_GPS_Lib_V01.h"

GPSLib gps;
bool gps_enable = false;
data_gps_t gps_current_data;

//===========================================
//  Accelerometer & Gyroscope & Magnetometer MPU9255 (GY-91) 10DOF
#include "ND_IMU_Lib_V01.h"

// MPU9250
IMULib imu_sensor;
bool imu_enable = false;
// uint32_t imu_counter_record = 0;
String imu_last_record = "";
String imu_last_Euler_Angles_record = "";

// BMP280
BMP280 bmp_sensor;
bool bmp_enable = false;

void setup() {
  // Serial Initialization
  Serial.begin(115200);
  Serial.println();
  Serial.println(MAIN_MESSAGE_INITIAL);
  Serial.println();

  // incrementa o contador de boot
  EEPROM.begin(2);
  boot_sequence = module.get_boot_sequence();
  Serial.print("Boot Sequence: ");
  Serial.println(boot_sequence);

  // Serial.println(__LINE__);

  // Creates a Task that will execute the TaskLoRa () function, with priority 1 and running in the nucleus 0
  xTaskCreatePinnedToCore(
                  TaskLoRa,       // Function with the code that implements the Task
                  "TaskLoRa",     // Task name
                  4096,           // Stack size to be allocated when creating the Task
                  NULL,           // Task input parameters
                  1,              // Task priority
                  &TaskIdLoRa,    // Reference to accompany the Task
                  MAIN_CORE_1);   // Core on which the Task will run (0 or 1 for ESP32)
  delay(100);                     // Delay for next command

  // Serial.println(__LINE__);

  // Creates a Task that will execute the TaskLoRa () function, with priority 1 and running in the nucleus 1
  xTaskCreatePinnedToCore(
                  TaskSDCard,     // Function with the code that implements the Task
                  "TaskSDCard",   // Task name
                  4096,           // Stack size to be allocated when creating the Task
                  NULL,           // Task input parameters
                  1,              // Task priority
                  &TaskIdSDCard,  // Reference to accompany the Task
                  MAIN_CORE_0);   // Core on which the Task will run (0 or 1 for ESP32)
  delay(100);                     // Delay for next command

  // Serial.println(__LINE__);

  // Creates a Task that will execute the TaskLoRa () function, with priority 1 and running in the nucleus 1
  xTaskCreatePinnedToCore(
                  TaskGPS,        // Function with the code that implements the Task
                  "TaskGPS",      // Task name
                  2048,           // Stack size to be allocated when creating the Task
                  NULL,           // Task input parameters
                  1,              // Task priority
                  &TaskIdGPS,     // Reference to accompany the Task
                  MAIN_CORE_0);   // Core on which the Task will run (0 or 1 for ESP32)
  delay(100);                     // Delay for next command

  // Serial.println(__LINE__);

    // Creates a Task that will execute the TaskLoRa () function, with priority 1 and running in the nucleus 0
  xTaskCreatePinnedToCore(
                  TaskIMU,        // Function with the code that implements the Task
                  "TaskIMU",      // Task name
                  4096,           // Stack size to be allocated when creating the Task
                  NULL,           // Task input parameters
                  1,              // Task priority
                  &TaskIdIMU,     // Reference to accompany the Task
                  MAIN_CORE_1);   // Core on which the Task will run (0 or 1 for ESP32)
  delay(100);                     // Delay for next command

  // Serial.println(__LINE__);

  // Enables the watchdog with a 15-second timeout
  esp_task_wdt_init(MAIN_WATCHDOC_TIME_OUT, true);
}

// Task for the LoRa radio module
void TaskLoRa( void * pvParameters ) {
  esp_task_wdt_add(NULL);

  // Serial.println(__LINE__);

  //===========================================
  // LoRa
  lora_enable = lora.begin(device_id);
  // ( !lora_enable ) ? Serial.println("Error initializing the LoRa module") : Serial.println("LoRa module OK!");
  long last_time = 0;
  lora_send_t message_REPLAY;

  uint16_t counter_reconnection = COUNTER_RECONNECTION;

  // Mandatory infinite loop to keep the Task running
  while( true ) {

    lora.run();

    if ( millis() > ( last_time + 5000 ) ) {
      Serial.println("TASK_NODE_LORA");
      last_time = millis();

      if ( !lora.is_connected() ) {
        lora_send_t message_JOIN;

        message_JOIN.sourcer_device_id = device_id;
        message_JOIN.target_device_id = gateway_id;
        message_JOIN.message_id = module.hash(device_id);
        message_JOIN.message_type = JOIN;

        lora.add_send_message(message_JOIN);

      }

      // decreventa 
      counter_reconnection--;


      // verifica se a tensao da bateria esta acima do minimo 7.00V se for
      // a tensao cair abaixo por 20 verificacoes o modulo entra em modo deepsleep
      module.checkBattery();
    }

    if ( !counter_reconnection ) {

      // Serial.println(__LINE__);
      
      lora.set_connected(false);
    }

    if ( lora.is_recv_list_empty() ) {
      lora_recv_t message_recv = lora.get_next_recv_message();

      uint64_t request_epochtime_now = now();
      if (gps_enable) {
        request_epochtime_now = gps.epochtime();
      }

      switch (message_recv.message_type) {
      case JOIN_ACK:

        lora.set_connected(true);
        gateway_id = message_recv.sourcer_device_id;

        break;
      
      case REQUEST:

        // Prepara o registro de resposta ao pedido de REQUEST
        message_REPLAY.sourcer_device_id = device_id;
        message_REPLAY.target_device_id = gateway_id;
        message_REPLAY.message_type = REPLAY;

        // Enviar resposta com dados do GPS
        if (gps_enable) {

          // gps_current_data = gps.get_record();      // pega dados do modulo gps

          // behavior_paramenters.gps_speed = gps_current_data.speed;
          // behavior_paramenters.gps_distanceBetweenTwoPoints = gps_current_data.distanceBetweenTwoPoints;
          // behavior_paramenters.gps_age = gps_current_data.age;

          message_REPLAY.payload = gps.get_string_record(gps_current_data, millis());
          message_REPLAY.message_id = module.hash(message_REPLAY.payload);

          lora.add_send_message(message_REPLAY);
          delay(100);

/*
          message_REPLAY.payload = gps.get_record(millis());      // pega dados do modulo gps
          message_REPLAY.message_id = module.hash(message_REPLAY.payload);

          lora.add_send_message(message_REPLAY);
          delay(100);
*/
        }

        // Enviar resposta com dados de Euler Angles do IMU
        if (imu_enable) {
          // Busca um Euler Angles e converte para uma string
          message_REPLAY.payload = imu_sensor.euler_angles_to_string(request_epochtime_now, millis(), imu_sensor.get_euler_angles(imu_sensor.get_quaternion()));
          // message_REPLAY.payload = imu_last_record;
          message_REPLAY.message_id = module.hash(message_REPLAY.payload);

          behavior_paramenters.imu_Pitch = imu_sensor.get_moving_average_pitch();

          lora.add_send_message(message_REPLAY);
          delay(100);
        }

        if (bmp_enable) {
          // busca a pressao e altitude
          message_REPLAY.payload = "[6," + String((unsigned long)(request_epochtime_now)) + "," + String(bmp_sensor.get_pressure_pa(), 2) + "," + String(bmp_sensor.get_altitude_meter(), 2) + "]";
          // message_REPLAY.payload = imu_last_record;
          message_REPLAY.message_id = module.hash(message_REPLAY.payload);

          lora.add_send_message(message_REPLAY);
          delay(100);
        }

        //
        message_REPLAY.payload = "[7," + String((unsigned long)(request_epochtime_now)) + "," + String(module.getVBat(), 2) + "]";
        message_REPLAY.message_id = module.hash(message_REPLAY.payload);
        lora.add_send_message(message_REPLAY);

        message_REPLAY.payload = "[8," + String(behavior) + "," + String(module.getVBat(), 2) + "]";
        message_REPLAY.message_id = module.hash(message_REPLAY.payload);
        lora.add_send_message(message_REPLAY);

        if ( !lora.is_connected() ) {
          lora.set_connected(true);
          gateway_id = message_recv.sourcer_device_id;
        }

        // zera o contador de perda conexao com o gateway
        counter_reconnection = COUNTER_RECONNECTION;

        break;
      
      default:
        break;
      }
    }

    if ( lora.is_send_list_empty() ) {
      lora.send_message( lora.get_next_send_message() );
    }

    esp_task_wdt_reset();                                   // Reset watchdog counter 
    vTaskDelay(pdMS_TO_TICKS(LORA_TASK_DELAY_MS));      // Pause Tesk and release the nucleus for the next Tesk in the priority queue
  }
}

// Task for the SDCard module
void TaskSDCard( void * pvParameters ) {
  esp_task_wdt_add(NULL);

  // Serial.println(__LINE__);

  // Initializes the SD Card module
  sdCard_enable = sdCard.begin();
  ( !sdCard_enable ) ? Serial.println("SD Card module initialization error") : Serial.println("SD Card Module OK!");

  // Mandatory infinite loop to keep the Task running
  while(true) {
    if ( sdCard_enable ) {

      sdCard.run();
    }

    esp_task_wdt_reset();                                       // Reset watchdog counter
    vTaskDelay(pdMS_TO_TICKS(SD_TASK_DELAY_MS));      // Pause Tesk and release the nucleus for the next Tesk in the priority queue
  }
}

// Task for the GPS module
void TaskGPS( void * pvParameters ) {
  esp_task_wdt_add(NULL);

  // Serial.println(__LINE__);

  gps_enable = gps.begin(device_id);
  // ( !gps_enable ) ? Serial.println("GPS module initialization error") : Serial.println("GPS module OK!");

  long gps_last_time = millis();

  while (true) {

    if ( gps_enable ) {

      gps.run();

      String gps_string_record;

      if ( gps.new_record() ) {

        gps_current_data = gps.get_record();      // pega dados do modulo gps

        behavior_paramenters.gps_speed = gps_current_data.speed;
        behavior_paramenters.gps_distanceBetweenTwoPoints = gps_current_data.distanceBetweenTwoPoints;
        behavior_paramenters.gps_age = gps_current_data.age;

        // Serial.println(gps.get_record(millis()));
        gps_string_record = gps.get_string_record(gps_current_data, millis());
        Serial.println(gps_string_record);

      }

      if ( sdCard.is_SDCard_present() ) {

        if ( millis() > ( gps_last_time + GPS_LAST_TIME ) ) {

          // String gps_payload = gps.get_record(millis());

          SDCard_record_t new_record;

          new_record.id = device_id;
          new_record.bootSequence = boot_sequence;
          new_record.type = TYPE_GPS;
          // new_record.payload = gps_payload;

          new_record.payload = gps_string_record;

          Serial.print("SDCD_APEN: ");
          // Serial.println(gps_payload);
          Serial.println(gps_string_record);
            
          sdCard.add_record(new_record);

          gps_last_time = millis();

        }
      }
    }
    
    esp_task_wdt_reset();                                   // Reset watchdog counter
    vTaskDelay(pdMS_TO_TICKS(GPS_TASK_DELAY_MS));        // Pause Tesk and release the nucleus for the next Tesk in the priority queue
  }

}

// Task for the GPS module
void TaskIMU( void * pvParameters ) {
  esp_task_wdt_add(NULL);

  // Serial.println(__LINE__);

  // Initializes the MPU9250 module
  imu_enable = imu_sensor.begin();
  ( !imu_enable ) ? Serial.println("Error initializing the IMU module MPU9250") : Serial.println("IMU module MPU9250 OK!");

  // Initializes the MPU9250 module
  bmp_enable = bmp_sensor.begin();
  ( !bmp_enable ) ? Serial.println("Error initializing the BMP280") : Serial.println("IMU module BMP280 OK!");

  // Mandatory infinite loop to keep the Task running
  while(true) {
    // Verifica se o imu estar conectado
    if ( imu_enable ) {
      // verifica se existe fluxo de dados na FIFO do imu
      if ( imu_sensor.avaliable() ) {
        // Verifica se todos os dados da FIFO sao validos e completos para formar um novo registro
        if ( imu_sensor.update_datas() ) {

          // Cria variavel para guarda a hora atual usando dados do gps ou da cpu
          time_t imu_epochtime_now = now();
          if (gps_enable) {
            imu_epochtime_now = gps.epochtime();
          }

          // pega a string com todos os dados do imu
          imu_last_record = imu_sensor.get_string_data_imu(imu_epochtime_now, millis(), boot_sequence);

          //
          if ( (behavior_paramenters.imu_Pitch > -60) && (behavior_paramenters.imu_Pitch < -35) ) {
            behavior = 0;
          }

          if ( (behavior_paramenters.gps_speed < 1.00) && (behavior_paramenters.gps_distanceBetweenTwoPoints == 0 && (behavior_paramenters.gps_age < 5000))) {
            behavior = 1;
          }

          if ( (behavior_paramenters.imu_Pitch > -20) && (behavior_paramenters.imu_Pitch < 30) ) {
            if ( behavior_paramenters.gps_speed >= 1.00 && behavior_paramenters.gps_age < 5000 ) {
              behavior = 2;
            }
          }

          if ( (behavior_paramenters.imu_Pitch > 35) && (behavior_paramenters.imu_Pitch < 70) ) {
            behavior = 3;
          }

          imu_last_record += ",";
          imu_last_record += String(behavior);
          imu_last_record += ",";
          imu_last_record += String(bmp_sensor.get_pressure_pa(), 2);
          imu_last_record += ",";
          imu_last_record += String(bmp_sensor.get_altitude_meter(), 2);
          imu_last_record += ",";
          imu_last_record += String(module.getVBat(), 2);

          // Verifica se o SDCard esta presente
          if ( sdCard_enable ) {
            // Cria uma nova variavel para armazenas as informacoes a serem salva no SDCard
            SDCard_record_t imu_new_record;
            imu_new_record.id = device_id;
            imu_new_record.bootSequence = boot_sequence;
            imu_new_record.type = TYPE_IMU;
            imu_new_record.payload = imu_last_record;

            // Adiciona um novo registro na lista de dados a serem salvo no SDCard
            sdCard.add_record(imu_new_record);
          }
        }
      }
    }

    esp_task_wdt_reset();                                       // Reset watchdog counter
    vTaskDelay(pdMS_TO_TICKS(IMU_TASK_DELAY_MS));      // Pause Tesk and release the nucleus for the next Tesk in the priority queue
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
