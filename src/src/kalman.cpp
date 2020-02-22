#include "fusion.hpp"

State::State(){

}

// This is updating our observation vector from out struct of IMU data
    //IMPORTANT--this wont work once we update our sensor suite
void State::dataAq(IMUdata *data){
    matrices::x_m << data->LINEAR_ACCEL[0],
                 data->LINEAR_ACCEL[1],
                 data->LINEAR_ACCEL[2],
                 data->GYRO[0],
                 data->GYRO[1],
                 data->GYRO[2],
                 data->MAG[0],
                 data->MAG[1],
                 data->MAG[2];   
}



Eigen::MatrixXd State::predict(){
    //x should already be defined

    //

 //Prediction step x = Ax
    matrices::x_kp = matrices::A * matrices::x_k_1;
}


void State::updateDynamics(){


    //rest of the kalman filter



// for the next thread iteration, we want that previous state to be whatever this one is    
matrices::x_k_1 = matrices::x_kp;


}

