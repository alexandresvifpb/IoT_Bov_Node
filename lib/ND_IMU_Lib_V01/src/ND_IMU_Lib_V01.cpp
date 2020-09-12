#include "ND_IMU_Lib_V01.h"

MPU9250_DMP imu;

LinkedList<EulerAngles_t> listEulerAngles = LinkedList<EulerAngles_t>();
boolean newDatasIMUAvaliable = false;

float imu_moving_average_yaw_array[IMU_MOVING_AVERAGE_ARRAY_SIZE];
float imu_moving_average_yaw_value = 0;
float imu_moving_average_pitch_array[IMU_MOVING_AVERAGE_ARRAY_SIZE];
float imu_moving_average_pitch_value = 0;
float imu_moving_average_roll_array[IMU_MOVING_AVERAGE_ARRAY_SIZE];
float imu_moving_average_roll_value = 0;

bool mpu9250_status = false;
bool bmp280 = false;
bool scanner(byte addres);

IMULib::IMULib() {}

// 
bool IMULib::begin(void) {

    // Call imu.begin() to verify communication and initialize
    if (imu.begin() != INV_SUCCESS) return false;

    // Use setSensors to turn on or off MPU-9250 sensors.
    // Any of the following defines can be combined:
    // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
    // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
    // Enable all sensors:
    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    // Use setGyroFSR() and setAccelFSR() to configure the
    // gyroscope and accelerometer full scale ranges.
    // Gyro options are +/- 250, 500, 1000, or 2000 dps
    imu.setGyroFSR(2000); // Set gyro to 2000 dps
    // Accel options are +/- 2, 4, 8, or 16 g
    imu.setAccelFSR(2); // Set accel to +/-2g
    // Note: the MPU-9250's magnetometer FSR is set at 
    // +/- 4912 uT (micro-tesla's)

    // setLPF() can be used to set the digital low-pass filter
    // of the accelerometer and gyroscope.
    // Can be any of the following: 188, 98, 42, 20, 10, 5
    // (values are in Hz).
    imu.setLPF(5); // Set LPF corner frequency to 5Hz

    // The sample rate of the accel/gyro can be set using
    // setSampleRate. Acceptable values range from 4Hz to 1kHz
    imu.setSampleRate(10); // Set sample rate to 10Hz

    // Likewise, the compass (magnetometer) sample rate can be
    // set using the setCompassSampleRate() function.
    // This value can range between: 1-100Hz
    imu.setCompassSampleRate(10); // Set mag rate to 10Hz

    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
                 DMP_FEATURE_GYRO_CAL, // Use gyro calibration
                 10); // Set DMP FIFO rate to 10 Hz
    // DMP_FEATURE_LP_QUAT can also be used. It uses the 
    // accelerometer in low-power mode to estimate quat's.
    // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

    // Manda uma mensagem pela porta I2C para o endereco do MPU-9250
    return scanner(IMU_MPU9250_ADDRESS);
}

// 
bool IMULib::avaliable(void) {
    newDatasIMUAvaliable = imu.fifoAvailable();
    return newDatasIMUAvaliable;
}

// 
bool IMULib::update_datas(void) {
    if (newDatasIMUAvaliable) {
        newDatasIMUAvaliable = false;
        // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
        if ( imu.dmpUpdateFifo() == INV_SUCCESS) {
            // computeEulerAngles can be used -- after updating the
            // quaternion values -- to estimate roll, pitch, and yaw
            imu.computeEulerAngles();
        }
        else return false;      // se não tiver dados disponivel na FIFO retorna false

        // dataReady() checks to see if new accel/gyro data
        // is available. It will return a boolean true or false
        // (New magnetometer data cannot be checked, as the library
        //  runs that sensor in single-conversion mode.)
        if ( imu.dataReady() ) {
            // Call update() to update the imu objects sensor data.
            // You can specify which sensors to update by combining
            // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
            // UPDATE_TEMPERATURE.
            // (The update function defaults to accel, gyro, compass,
            //  so you don't have to specify these values.)
            imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
        }
        return true;            // dados na FIFO disponivel
    } 
    else {
        return false;
    }
}

// 
IMU_t IMULib::get_imu(void) {
    IMU_t datasIMU;

    datasIMU.accelX = imu.calcAccel(imu.ax);
    datasIMU.accelY = imu.calcAccel(imu.ay);
    datasIMU.accelZ = imu.calcAccel(imu.az);

    datasIMU.gyroX = imu.calcGyro(imu.gx);
    datasIMU.gyroY = imu.calcGyro(imu.gy);
    datasIMU.gyroZ = imu.calcGyro(imu.gz);

    datasIMU.magX = imu.calcMag(imu.mx);
    datasIMU.magY = imu.calcMag(imu.my);
    datasIMU.magZ = imu.calcMag(imu.mz);

    return datasIMU;
}

// 
Quaternion_t IMULib::get_quaternion(void) {
    Quaternion_t datasQuaternion;

    datasQuaternion.q0 = imu.calcQuat(imu.qw);
    datasQuaternion.q1 = imu.calcQuat(imu.qx);
    datasQuaternion.q2 = imu.calcQuat(imu.qy);
    datasQuaternion.q3 = imu.calcQuat(imu.qz);

    return datasQuaternion;
}

// 
EulerAngles_t IMULib::get_euler_angles(Quaternion_t quaternion) {
    EulerAngles_t eulerAngles;

    // double ysqr = y * y;
    double ysqr = quaternion.q2 * quaternion.q2;

    // roll (x-axis rotation)

    // double t0 = +2.0 * (w * x + y * z);
    // double t1 = +1.0 - 2.0 * (x * x + ysqr);
    // roll = atan2(t0, t1);

    double t0 = +2.0 * (quaternion.q0 * quaternion.q1 + quaternion.q2 * quaternion.q3);
    double t1 = +1.0 - 2.0 * (quaternion.q1 * quaternion.q1 + ysqr);
    eulerAngles.Roll = atan2(t0, t1) * (180.0 / PI);

    // pitch (y-axis rotation)

    // double t2 = +2.0 * (w * y - z * x);
    // t2 = t2 > 1.0 ? 1.0 : t2;
    // t2 = t2 < -1.0 ? -1.0 : t2;
    // pitch = asin(t2);

    double t2 = +2.0 * (quaternion.q0 * quaternion.q2 - quaternion.q3 * quaternion.q1);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    eulerAngles.Pitch = asin(t2) * (180.0 / PI);

    // yaw (z-axis rotation)

    // double t3 = +2.0 * (w * z + x * y);
    // double t4 = +1.0 - 2.0 * (ysqr + z * z);  
    // yaw = atan2(t3, t4);

    double t3 = +2.0 * (quaternion.q0 * quaternion.q3 + quaternion.q1 * quaternion.q2);
    double t4 = +1.0 - 2.0 * (ysqr + quaternion.q3 * quaternion.q3);  
    eulerAngles.Yaw = atan2(t3, t4) * (180.0 / PI);

    return eulerAngles;
}

//
uint8_t IMULib::add_Euler_Angle_list(EulerAngles_t eulerAngle) {
    listEulerAngles.add(eulerAngle);
    return listEulerAngles.size();
}

// 
String IMULib::get_string_data_imu(uint64_t epochtime, uint64_t nowtime, uint16_t bootsequence) {
    IMU_t datasIMU = get_imu();
    Quaternion_t datasQuaternion = get_quaternion();
    EulerAngles_t datasEulerAngle = get_euler_angles(datasQuaternion);

    String result = String((unsigned long)epochtime);
    result += ",";
    result += String((unsigned long)nowtime);
    result += ",";
    result += bootsequence;
    result += ",";
    result += get_str_imu(datasIMU);
    result += ",";
    result += get_string_quaternion(datasQuaternion);
    result += ",";
    result += get_string_Euler_Angles(datasEulerAngle);
    result += ",";
    result += imu_moving_average_yaw(datasEulerAngle.Yaw);
    result += ",";
    result += imu_moving_average_pitch(datasEulerAngle.Pitch);
    result += ",";
    result += imu_moving_average_roll(datasEulerAngle.Roll);
    result += ",";
    result += imu_classify_behavior();
    return result;
}

// 
String IMULib::get_string_data_list_Euler_Angle(uint64_t epochtime, uint64_t nowtime, uint16_t bootsequence) {

    String result = String((unsigned long)epochtime);
    result += ",";
    result += String((unsigned long)nowtime);
    result += ",";
    result += bootsequence;
    result += ",";

    while ( listEulerAngles.size() ) {
        EulerAngles_t datasEulerAngle = listEulerAngles.remove(0);
        result += ",";
        result += get_string_Euler_Angles(datasEulerAngle);
    }
    
    return result;
}

// 
String IMULib::get_str_imu(IMU_t datasIMU) {
    String result;
    result += String(datasIMU.accelX,4);
    result += ",";
    result += String(datasIMU.accelY,4);
    result += ",";
    result += String(datasIMU.accelZ,4);
    result += ",";
    result += String(datasIMU.gyroX,4);
    result += ",";
    result += String(datasIMU.gyroY,4);
    result += ",";
    result += String(datasIMU.gyroZ,4);
    result += ",";
    result += String(datasIMU.magX,4);
    result += ",";
    result += String(datasIMU.magY,4);
    result += ",";
    result += String(datasIMU.magZ,4);
    return result;
}

// 
String IMULib::get_string_quaternion(Quaternion_t datasQuaternion) {
    String result;
    result += String(datasQuaternion.q0,4);
    result += ",";
    result += String(datasQuaternion.q1,4);
    result += ",";
    result += String(datasQuaternion.q2,4);
    result += ",";
    result += String(datasQuaternion.q3,4);
    return result;
}

// 
String IMULib::get_string_Euler_Angles(EulerAngles_t datasEulerAngles) {
    String result;
    result += String(datasEulerAngles.Yaw,4);
    result += ",";
    result += String(datasEulerAngles.Pitch,4);
    result += ",";
    result += String(datasEulerAngles.Roll,4);
    return result;
}

//
String IMULib::get_string_board_display_naive(void) {
  String result;
  int sizeBuffer = 18;

  byte* gyroDataX = (byte*)(imu.gx);
  byte* gyroDataY = (byte*)(imu.gy);
  byte* gyroDataZ = (byte*)(imu.gz);

  byte* accelDataX = (byte*)(imu.ax);
  byte* accelDataY = (byte*)(imu.ay);
  byte* accelDataZ = (byte*)(imu.az);

  byte* magDataX = (byte*)(imu.mx);
  byte* magDataY = (byte*)(imu.my);
  byte* magDataZ = (byte*)(imu.mz);

  byte buf[sizeBuffer] = {gyroDataX[0],gyroDataX[1],
                  gyroDataY[0],gyroDataY[1],
                  gyroDataZ[0],gyroDataZ[1],

                  accelDataX[0],accelDataX[1],
                  accelDataY[0],accelDataY[1],
                  accelDataZ[0],accelDataZ[1],

                  magDataX[0],magDataX[1],
                  magDataY[0],magDataY[1],
                  magDataZ[0],magDataZ[1]};

  Serial.println("-368,7494,1818,10989,41,13646,-3791,3696,739");

  return result;
}

//
String IMULib::get_string_NG_imu(uint8_t type) {
  String result;
  EulerAngles_t eulerAngle = get_euler_angles(get_quaternion());

  switch (type) {
  case 0:
    result = "/sensors, ";
    result += imu.calcAccel(imu.gx);
    result += ", ";
    result += imu.calcAccel(imu.gy);
    result += ", ";
    result += imu.calcAccel(imu.gz);
    result += ", ";
    result += imu.calcAccel(imu.ax);
    result += ", ";
    result += imu.calcAccel(imu.ay);
    result += ", ";
    result += imu.calcAccel(imu.az);
    result += ", ";
    result += imu.calcAccel(imu.mx);
    result += ", ";
    result += imu.calcAccel(imu.my);
    result += ", ";
    result += imu.calcAccel(imu.mz);
    result += ", ";
    result += "1.00";
    result += "\r\n";
    break;
  
  case 1:
    result = "/quaternion, ";
    result += imu.calcQuat(imu.qw);
    result += ", ";
    result += imu.calcQuat(imu.qx);
    result += ", ";
    result += imu.calcQuat(imu.qy);
    result += ", ";
    result += imu.calcQuat(imu.qz);
    result += "\r\n";
    break;
  
  case 2:
    result = "/euler, ";
    result += eulerAngle.Roll;
    result += ", ";
    result += eulerAngle.Pitch;
    result += ", ";
    result += eulerAngle.Yaw;
    result += "\r\n";
    break;
  
  default:
    break;
  }
  
  return result;
}

//
String IMULib::get_JSON_MPU9250(uint8_t type) {
  String result;
  EulerAngles_t eulerAngle = get_euler_angles(get_quaternion());

  switch (type) {
  case 0:
    result = "{\"type\":\"all\",\"payload\":[";
    result += imu.calcAccel(imu.gx);
    result += ", ";
    result += imu.calcAccel(imu.gy);
    result += ", ";
    result += imu.calcAccel(imu.gz);
    result += ", ";
    result += imu.calcAccel(imu.ax);
    result += ", ";
    result += imu.calcAccel(imu.ay);
    result += ", ";
    result += imu.calcAccel(imu.az);
    result += ", ";
    result += imu.calcAccel(imu.mx);
    result += ", ";
    result += imu.calcAccel(imu.my);
    result += ", ";
    result += imu.calcAccel(imu.mz);
    result += ", ";
    result += imu.calcQuat(imu.qw);
    result += ", ";
    result += imu.calcQuat(imu.qx);
    result += ", ";
    result += imu.calcQuat(imu.qy);
    result += ", ";
    result += imu.calcQuat(imu.qz);
    result += ", ";
    result += eulerAngle.Roll;
    result += ", ";
    result += eulerAngle.Pitch;
    result += ", ";
    result += eulerAngle.Yaw;
    result += "]}";
    break;
  
  case 1:
    result = "{\"type\":\"sensors\",\"payload\":[";
    result += imu.calcAccel(imu.gx);
    result += ", ";
    result += imu.calcAccel(imu.gy);
    result += ", ";
    result += imu.calcAccel(imu.gz);
    result += ", ";
    result += imu.calcAccel(imu.ax);
    result += ", ";
    result += imu.calcAccel(imu.ay);
    result += ", ";
    result += imu.calcAccel(imu.az);
    result += ", ";
    result += imu.calcAccel(imu.mx);
    result += ", ";
    result += imu.calcAccel(imu.my);
    result += ", ";
    result += imu.calcAccel(imu.mz);
    result += ", ";
    result += "1.00";
    result += "]}";
    break;
  
  case 2:
    result = "{\"type\":\"quaternion\",\"payload\":[";
    result += imu.calcQuat(imu.qw);
    result += ", ";
    result += imu.calcQuat(imu.qx);
    result += ", ";
    result += imu.calcQuat(imu.qy);
    result += ", ";
    result += imu.calcQuat(imu.qz);
    result += "]}";
    break;
  
  case 3:
    result = "{\"type\":\"euler\",\"payload\":[";
    result += eulerAngle.Roll;
    result += ", ";
    result += eulerAngle.Pitch;
    result += ", ";
    result += eulerAngle.Yaw;
    result += "]}";
    break;
  
  default:
    break;
  }
  
  return result;
}

// gabiarra para retorna os dados do Euler Angle
String IMULib::euler_angles_to_string(uint64_t epochtime, uint64_t nowtime, EulerAngles_t euler_angle) {
  String _euler_angle_string = "";

  _euler_angle_string = "[";
  _euler_angle_string += IMU_TYPE_EA;
  _euler_angle_string += ",";
  _euler_angle_string += String((unsigned long)epochtime);
  _euler_angle_string += ",";
  _euler_angle_string += String((unsigned long)nowtime);
  _euler_angle_string += ",";
  _euler_angle_string += euler_angle.Roll;
  _euler_angle_string += ",";
  _euler_angle_string += euler_angle.Pitch;
  _euler_angle_string += ",";
  _euler_angle_string += euler_angle.Yaw;
  _euler_angle_string += ",";
  _euler_angle_string += imu_classify_behavior();
  _euler_angle_string += "]";

  return _euler_angle_string;
}

bool scanner(byte address) {

  byte error;

  Serial.print("I2C scanner address: ");
  Serial.print(" (0x");
  Serial.print(address, HEX);
  Serial.println(")");

  Wire.beginTransmission(address);
  error = Wire.endTransmission();

  if ( error == 0 ) {
    Serial.println("SUCCESS!!!");
    return true;
  }
  Serial.println("FALL!!!");
  return false;
}


// Private


//
float IMULib::imu_moving_average_yaw(float value) {
  float _imu_accumulator = 0;
  for (int8_t i = IMU_MOVING_AVERAGE_ARRAY_SIZE-1; i > 0; i--) {
    imu_moving_average_yaw_array[i] = imu_moving_average_yaw_array[i-1];
    _imu_accumulator += imu_moving_average_yaw_array[i];
  }

  imu_moving_average_yaw_array[0] = value;
  _imu_accumulator += value;

  imu_moving_average_yaw_value = _imu_accumulator/IMU_MOVING_AVERAGE_ARRAY_SIZE;
  return imu_moving_average_yaw_value;
}

//
float IMULib::imu_moving_average_pitch(float value) {
  float _imu_accumulator = 0;
  for (int8_t i = IMU_MOVING_AVERAGE_ARRAY_SIZE-1; i > 0; i--) {
    imu_moving_average_pitch_array[i] = imu_moving_average_pitch_array[i-1];
    _imu_accumulator += imu_moving_average_pitch_array[i];
  }

  imu_moving_average_pitch_array[0] = value;
  _imu_accumulator += value;

  imu_moving_average_pitch_value = _imu_accumulator/IMU_MOVING_AVERAGE_ARRAY_SIZE;
  return imu_moving_average_pitch_value;
}

//
float IMULib::imu_moving_average_roll(float value) {
  float _imu_accumulator = 0;
  for (int8_t i = IMU_MOVING_AVERAGE_ARRAY_SIZE-1; i > 0; i--) {
    imu_moving_average_roll_array[i] = imu_moving_average_roll_array[i-1];
    _imu_accumulator += imu_moving_average_roll_array[i];
  }

  imu_moving_average_roll_array[0] = value;
  _imu_accumulator += value;

  imu_moving_average_roll_value = _imu_accumulator/IMU_MOVING_AVERAGE_ARRAY_SIZE;
  return imu_moving_average_roll_value;
}

//
uint8_t IMULib::imu_classify_behavior(void) {
  uint8_t rest = 0;
  if ( imu_moving_average_pitch_value > -20 && imu_moving_average_pitch_value < 30) rest = 1;
  if ( imu_moving_average_pitch_value > 35 && imu_moving_average_pitch_value < 70) rest = 2;
  if ( imu_moving_average_pitch_value > -60 && imu_moving_average_pitch_value < -35) rest = 3;
  return rest;
}


/**
 * Class BMP280
 **/
BME280 bmp280_sensor;

//
BMP280::BMP280() {}

//
bool BMP280::begin(void) {
  bool _enable = false;

  bmp280_sensor.settings.commInterface = I2C_MODE;
  bmp280_sensor.settings.I2CAddress = BMP_ADDRESS;
  bmp280_sensor.settings.runMode = 3; //Normal mode
  bmp280_sensor.settings.tStandby = 0;
  bmp280_sensor.settings.filter = 4;
  bmp280_sensor.settings.tempOverSample = 5;
  bmp280_sensor.settings.pressOverSample = 5;
  bmp280_sensor.settings.humidOverSample = 5;

  delay(10);      //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  byte bmp_id = bmp280_sensor.begin();

  Serial.print("BMP_id = ");
  Serial.println(bmp_id);

  if ( bmp_id != 0x58 ) {
    _enable = false;
  } else {
    _enable = true;
  }

  return _enable;  
}

//
float BMP280::get_pressure_pa(void) {
  // return bmp.readPressure();
  return bmp280_sensor.readFloatPressure();
}

//
float BMP280::get_altitude_meter(void) {
  // return bmp.readAltitude(BMP_ADJUSTED_LOCAL_ALTUTUDE);
  return bmp280_sensor.readFloatAltitudeMeters();
}