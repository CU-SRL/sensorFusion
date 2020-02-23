#include "fusion.hpp"

DigitalIMU::DigitalIMU() {
    board = Adafruit_BNO055(55, 0x28);
}

DigitalIMU::DigitalIMU(int32_t sensorID, uint8_t address) {
    board = Adafruit_BNO055(sensorID, address);
}

bool DigitalIMU::begin() {
    if (board.begin()) {
        board.enableAutoRange(true); // Hopefully this will enable more than +/-4g???
        return true;
    }
    else {
        return false;
    }
}

void DigitalIMU::sample(IMUdata* data, State *ptr) {
    // Get data and store it to the imu_data struct

    // Linear Acceleration data [m/s^2]
    board.getEvent(&event,Adafruit_BNO055::VECTOR_LINEARACCEL);
    data->LINEAR_ACCEL[0] = event.acceleration.x;
    data->LINEAR_ACCEL[1] = event.acceleration.y;
    data->LINEAR_ACCEL[2] = event.acceleration.z;

    // Serial.printf(" l_accel_X: %.5f",data->LINEAR_ACCEL[0]);
    // Serial.printf(" l_accel_Y: %.5f",data->LINEAR_ACCEL[1]);
    // Serial.printf(" l_accel_Z: %.5f\n",data->LINEAR_ACCEL[2]);

    // Acceleration [milli-G]<->[mg]
    board.getEvent(&event,Adafruit_BNO055::VECTOR_ACCELEROMETER);
    data->GRAVITY_ACCEL[0] = event.acceleration.x;
    data->GRAVITY_ACCEL[1] = event.acceleration.y;
    data->GRAVITY_ACCEL[2] = event.acceleration.z;

    // Serial.printf(" g_accel_X: %.5f",data->GRAVITY_ACCEL[0]);
    // Serial.printf(" g_accel_Y: %.5f",data->GRAVITY_ACCEL[1]);
    // Serial.printf(" g_accel_Z: %.5f\n",data->GRAVITY_ACCEL[2]);

    // Gyro data [Degrees per second]<->[DPS] 
    board.getEvent(&event,Adafruit_BNO055::VECTOR_GYROSCOPE);
    data->GYRO[0] = event.gyro.x;
    data->GYRO[1] = event.gyro.y;
    data->GYRO[2] = event.gyro.z;

    // Serial.printf(" gyro_X: %.5f",data->GYRO[0]);
    // Serial.printf(" gyro_Y: %.5f",data->GYRO[1]);
    // Serial.printf(" gyro_Z: %.5f\n",data->GYRO[2]);

    // Magnetometer Data [micro-Tesla]<->[uT]
    board.getEvent(&event,Adafruit_BNO055::VECTOR_MAGNETOMETER);
    data->MAG[0] = event.magnetic.x;
    data->MAG[1] = event.magnetic.y;
    data->MAG[2] = event.magnetic.z;


    data->Temp = board.getTemp();

    // Serial.printf(" Magnetometer_X: %.5f",data->MAG[0]);
    // Serial.printf(" Magnetometer_Y: %.5f",data->MAG[1]);
    // Serial.printf(" Magnetometer_Z: %.5f\n",data->MAG[2]);
    // Serial.println("");

    imu::Quaternion quat;
    quat = board.getQuat();
    data->Quat[0] = quat.w();
    data->Quat[1] = quat.x();
    data->Quat[2] = quat.y();
    data->Quat[3] = quat.z();

    Serial.printf(" IMU_Quat_W: %.5f",data->Quat[0]);
    Serial.printf(" IMU_Quat_X: %.5f",data->Quat[1]);
    Serial.printf(" IMU_Quat_Y: %.5f\n",data->Quat[2]);
    Serial.printf(" IMU_Quat_Z: %.5f\n",data->Quat[3]);

    ptr->quaternion(ptr->eulerAngle(data->GYRO[0], constants::gyro_sen, constants::dt), 
                    ptr->eulerAngle(data->GYRO[1], constants::gyro_sen, constants::dt),
                    ptr->eulerAngle(data->GYRO[2], constants::gyro_sen, constants::dt),
                    data->q_w, data->q_x, data->q_y, data->q_z);

    Serial.printf(" Quat_W: %.5f",data->q_w);
    Serial.printf(" Quat_X: %.5f",data->q_x);
    Serial.printf(" Quat_Y: %.5f\n",data->q_y);
    Serial.printf(" Quat_Z: %.5f\n",data->q_z);

    data->t = millis();
}