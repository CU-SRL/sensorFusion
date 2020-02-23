/**
 * @file main.cpp
 * @brief The main source file for the Code Base
 * 
 * Furthermore, the void setup() and void loop() functions are defined here also.
 * 
 */

/***************************************************************/
/*                                                             */
/*                                                             */
/*                    CU-SRL AVIONICS TEAM                     */
/*            STATE ESTIMATION OF A DYNAMIC VEHICLE            */
/*                                                             */
/*                       Lyon Foster                           */
/*                       Jason Popich                          */
/*                   Madisen Purifoy-Frie                      */
/*                       Daniel Crook                          */
/*                                                             */
/***************************************************************/
#include "fusion.hpp"


// GLOBAL VARIABLES I KNOW BAD... FIX LATER
DigitalIMU IMU = DigitalIMU(55,0x28);
IMUdata imu_data;
State *state;

ThreadController thread_control = ThreadController();
Thread* ThreadIMU = new Thread(); 

void IMU_LOOP() {
    IMU.sample(&imu_data, state); /*!< Sample the IMU by calling the IMU Sample function */
}

void KILLSYSTEM()
{
    while(true)
    {
        Serial.println("Failed to Init!!");
    }
}

void setup()
{
    delay(5000); /*!< Wait 2.5 seconds before starting everything up */

    Serial.begin(115200); /*!< Start serial comms */

    Serial.println("Hola1");

    /* Initialize BNO055 IMU sensor */
    if (!IMU.begin()) {
        KILLSYSTEM();
    }

    state = new State(&imu_data);

    Serial.println("Hola2");

    ThreadIMU->onRun(IMU_LOOP); /*!< Set the IMU looping function for the ThreadController */
    ThreadIMU->setInterval(constants::interval_IMU); /*!< Set the IMU refresh rate (Interval) */


    /* Add threads to ThreadController */
    thread_control.add(ThreadIMU);

    // while (1 = 1)
    // {
    //     if (sqrt(imu_data.LINEAR_ACCEL[0]^2 + imu_data.LINEAR_ACCEL[1]^2 + imu_data.LINEAR_ACCEL[2]^2) > 20)
    //     {
    //         break;
    //     }  
    // } 
}

void loop()
{
    thread_control.run();
}