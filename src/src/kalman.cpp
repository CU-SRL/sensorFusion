#include "fusion.hpp"

// Constructor
State::State()
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
}

// Destructor
State::~State()
{

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


void State::updateDynamics()
{
    //rest of the kalman filter

    // for the next thread iteration, we want that previous state to be whatever this one is    
    matrices::x_k_1 = matrices::x_kp;
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

