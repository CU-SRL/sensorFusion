#include "fusion.hpp"

// Constructor
State::State(IMUdata* inputData) : data(inputData)
{
    /*********************/
    /* POPULATE A MATRIX */
    /*********************/

    // Declare and Define the 3 different matrix types for A all of which are 3x3's
    Eigen::Matrix3d delta_t; // (delta t) diagonal
    double f = constants::dt;
    delta_t << f, 0, 0,
               0, f, 0,
               0, 0, f;

    Eigen::Matrix3d delta_t_2; // (1/2)(delta t)^2 diagonal
    f = (0.5)*pow(constants::dt, 2);
    delta_t_2 << f, 0, 0,
                 0, f, 0,
                 0, 0, f;

    Eigen::Matrix3d delta_t_3; // 1/(delta t) diagonal
    f = 1/(constants::dt);
    delta_t_3 << f, 0, 0,
                 0, f, 0,
                 0, 0, f;

    // (delta t) diagonal blocks placed
    matrices::A.block<3,3>(0,3) = delta_t;
    matrices::A.block<3,3>(3,6) = delta_t;
    matrices::A.block<3,3>(9,12) = delta_t;

    // (1/2)(delta t)^2 diagonal blocks placed
    matrices::A.block<3,3>(0,6) = delta_t_2;
    
    // 1/(delta t) diagonal blocks placed
    matrices::A.block<3,3>(15,12) = delta_t_3;

    // State::print_mtxd(matrices::A);

/************************************************************************************/

    /*********************/
    /* POPULATE H MATRIX */
    /*********************/

     Eigen::Matrix3d I_block; //block of 3x3 indentity that will make populating H (the obvervation mapping matrix) way easier
     I_block <<  1, 0, 0,
                0, 1, 0,
                0, 0, 1;
    //Inserting Identity blocks
    matrices::H.block<3,3>(6,6) = I_block;
    matrices::H.block<3,3>(12,12) = I_block;
    matrices::H.block<3,3>(18,18) = I_block;
    
    State::print_mtxd(matrices::H);

}

// Destructor
State::~State()
{

}

// This is updating our observation vector from out struct of IMU data
    //IMPORTANT--this wont work once we update our sensor suite
void State::dataAq(IMUdata *data){
    matrices::x_m << 
                0,
                0,
                0,
                0,
                0,
                0,
                data->LINEAR_ACCEL[0],
                data->LINEAR_ACCEL[1],
                data->LINEAR_ACCEL[2],
                0,
                0,
                0,
                data->GYRO[0],
                data->GYRO[1],
                data->GYRO[2],
                0,
                0,
                0,
                data->MAG[0],
                data->MAG[1],
                data->MAG[2]; 

    matrices::y << 
                0,
                0,
                0,
                0,
                0,
                0,
                (data->LINEAR_ACCEL[0])*(1-State::calcAccelSystematicError()),
                (data->LINEAR_ACCEL[1])*(1-State::calcAccelSystematicError()),
                (data->LINEAR_ACCEL[2])*(1-State::calcAccelSystematicError()),
                0,
                0,
                0,
                (data->GYRO[0])*(1-State::calcGyroSystematicError()),
                (data->GYRO[1])*(1-State::calcGyroSystematicError()),
                (data->GYRO[2])*(1-State::calcGyroSystematicError()),
                0,
                0,
                0,
                data->MAG[0],
                data->MAG[1],
                data->MAG[2]; 
}

void State::predict(){
   /*This function takes our previous state x_k_1 and extrapolates it forward in time
   by dt using A which is our state transition matrix yeilding x_kp which is our state prediction
   without measurment correction*/
    matrices::x_kp = matrices::A * matrices::x_k_1;
}

void State::calculateKalmanGain()
{
    /*********************/
    /* POPULATE R MATRIX */
    /*********************/
    Eigen::VectorXd temp(21);
    temp << 
        0,
        0,
        0,
        0,
        0,
        0,
        (constants::baseAccel_error),
        (constants::baseAccel_error),
        (constants::baseAccel_error),
        0,
        0,
        0,
        (constants::baseGyro_error),
        (constants::baseGyro_error),
        (constants::baseGyro_error),
        0,
        0,
        0,
        0,
        0,
        0;

    matrices::R = temp.transpose()*temp;
    matrices::K = (matrices::P_kp*matrices::H)*((matrices::H*matrices::P_kp*matrices::H.transpose()+matrices::R).inverse());
}

void State::processCovarianceMatrix()
{
    matrices::P_kp = matrices::A * matrices::P_k_1 * matrices::A.transpose();
}

void State::updateDynamics()
{
    State::dataAq(State::data); // ACQUIRE DATA
    State::predict(); // 
    State::processCovarianceMatrix(); // 
    State::calculateKalmanGain(); // CALCULATE KALMAN GAIN
}

void State::print_mtxd(const Eigen::MatrixXd& X)  
{
   int i, j, nrow, ncol;
   
   nrow = X.rows();
   ncol = X.cols();

   Serial.print("nrow: "); Serial.println(nrow);
   Serial.print("ncol: "); Serial.println(ncol);       
   Serial.println();
   
   for (i=0; i<nrow; i++)
   {
       for (j=0; j<ncol; j++)
       {
           Serial.print(X(i,j), 6);   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();
}

float State::calcAccelSystematicError()
{
    return 0.03*((data->Temp)-25.0);
}

float State::calcGyroSystematicError()
{
    return 0.03*((data->Temp)-22.5);
}

