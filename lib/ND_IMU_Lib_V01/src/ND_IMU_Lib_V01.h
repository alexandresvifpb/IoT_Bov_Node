#ifndef ND_IMU_Lib_V01_h
#define ND_IMU_Lib_V01_h

#include "Arduino.h"
#include <SparkFunMPU9250-DMP.h>
#include "SparkFunBME280.h"
// #include <Adafruit_BMP280.h>
#include "LinkedList.h"

// MPU9250
// #define MPU_ID                       12345     // define um ID para o modulo
#define IMU_MPU9250_ADDRESS             0x68      //    0x69      // Device address when ADO = 0
#define IMU_PIN_SDA                     21        //(36)     //(GPIO_NUM_10)
#define IMU_PIN_SCL                     22        //(39)     //(GPIO_NUM_9)

#define ACGYMG_REPORTING_PERIOD_MS      20
#define DATA_ACCEL_DIF                  0.2
#define DATA_YAW_DIF                    0.2
#define DATA_PITCH_DIF                  0.2
#define DATA_ROLL_DIF                   0.2

#define IMU_TYPE_ALL                    2
#define IMU_TYPE_ACCEL                  3
#define IMU_TYPE_QUAT                   4
#define IMU_TYPE_EA                     5

#define MADGWICK_FILTER                 0         // MadgwickQuaternionUpdate()
#define MAHONY_FILTER                   1         // MahonyQuaternionUpdate()

// BMP280
#define BMP_ADDRESS                     0x76      //    0x77      // Device address when ADO = 0
#define BMP_ADJUSTED_LOCAL_ALTUTUDE     1013.25
#define BMP_TYPE_PRESS                  6

#define IMU_TASK_DELAY_MS               190

#define IMU_MOVING_AVERAGE_ARRAY_SIZE   10

typedef struct 
{
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    uint16_t magX;
    uint16_t magY;
    uint16_t magZ;
} IMU_t;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_t;

typedef struct {
    float Yaw;
    float Pitch;
    float Roll;
} EulerAngles_t;

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0,         // 0.6 mG per LSB
  MFS_16BITS              // 0.15 mG per LSB
};

enum DataStringIMU {
  DSI_ACGYMGT = 0,        // Accel, Gyro, Mag, Temperature
  DSI_QTEA                // Quaternion, EulerAngles
};

class IMULib {

  public:
    IMULib();

    bool begin(void);
    bool avaliable(void);
    bool update_datas(void);

    IMU_t get_imu(void);
    Quaternion_t get_quaternion(void);
    EulerAngles_t get_euler_angles(Quaternion_t quaternion);
    uint8_t add_Euler_Angle_list(EulerAngles_t eulerAngle);
    String get_string_data_imu(uint64_t epochtime, uint64_t nowtime, uint16_t bootsequence);
    String get_string_data_list_Euler_Angle(uint64_t epochtime, uint64_t nowtime, uint16_t bootsequence);
    
    String get_str_imu(IMU_t datasIMU);
    String get_string_quaternion(Quaternion_t datasQuaternion);
    String get_string_Euler_Angles(EulerAngles_t datasEulerAngles);

    String get_string_board_display_naive(void);
    String get_string_NG_imu(uint8_t type);
    String get_JSON_MPU9250(uint8_t type);

    String euler_angles_to_string(uint64_t epochtime, uint64_t nowtime, EulerAngles_t euler_angle);           // gabiarra para retorna apenas os dados do Euler Angle

    float get_moving_average_pitch(void);

    private:

    float imu_moving_average_yaw(float value);
    float imu_moving_average_pitch(float value);
    float imu_moving_average_roll(float value);

    uint8_t imu_classify_behavior(void);

};

class BMP280 {

  public:
    BMP280();

    bool begin(void);

    float get_pressure_pa(void);
    float get_altitude_meter(void);
    
    private:

    // float bmp_moving_average_pressure(float value);
    // float bmp_moving_average_altitude(float value);

};



#endif  // ND_IMU_Lib_V01_h