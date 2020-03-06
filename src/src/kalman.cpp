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
    // f = 1/(constants::dt);
    f = 0;
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
    matrices::H.block<3,3>(0,6) = I_block;
    matrices::H.block<3,3>(3,12) = I_block;
    // matrices::H.block<3,3>(6,18) = I_block;
    
    // State::print_mtxd(matrices::H);
}

// Destructor
State::~State(){}

// This is updating our observation vector from out struct of IMU data
    //IMPORTANT--this wont work once we update our sensor suite


void State::dataAq(IMUdata *data){
    /*******************************************/
    /* READ AND INTERPRET ACCEL AND GYRO DATA  */
    /*******************************************/

    /*
    NOTE: Currently, all coordinate transformations are bypassed to isolate accelerometer data.
        We still need to apply a finite impulse response (FIR) filter to remove high-frequency
        sensor noise, before running data through Kalman filter.
    */

    //Lets check this with how we have the IMU oriented at launch
    //all in rads/s or rads
    
    double p =  (data->GYRO[0])*(1-State::calcGyroSystematicError()) * PI/180;//Roll rate
    double q =  (data->GYRO[1])*(1-State::calcGyroSystematicError()) * PI/180;//Pitch rate
    double r =  (data->GYRO[2])*(1-State::calcGyroSystematicError()) * PI/180;//Yaw rate

    // Serial.print(p);
    // Serial.print(" ");
    // Serial.print(q);
    // Serial.print(" ");
    // Serial.print(r);
    // Serial.println(" ");

    double phi   = matrices::x_k_1.coeff(9);    // pulling these euler angles from the last state matrix 
    double theta = matrices::x_k_1.coeff(10);   // pulling these euler angles from the last state matrix
    double psi   = matrices::x_k_1.coeff(11);   // pulling these euler angles from the last state matrix

    double phi_k1 = 0.0;    //  Phi   at K+1
    double theta_k1 = 0.0;  //  Theta at K+1
    double psi_k1 = 0.0;    //  PSI   at K+1

    // Serial.print(phi);
    // Serial.print(" ");
    // Serial.print(theta);
    // Serial.print(" ");
    // Serial.print(psi);
    // Serial.print(" ");

    double phi_dot    = p + (q*sin(phi) + r*cos(phi)) * tan(theta);  // These are all of our angular rates relative to earth: phi dot
    double theta_dot  = q*cos(phi)-r*sin(phi);                       // These are all of our angular rates relative to earth: theta dot
    double psi_dot    = (q*sin(phi)+r*cos(phi))/cos(theta);          // These are all of our angular rates relative to earth: psi dot

    // Serial.print(phi_dot);
    // Serial.print(" ");
    // Serial.print(theta_dot);
    // Serial.print(" ");
    // Serial.println(psi_dot);

    // BYPASS DCM, see NOTE
    Eigen::VectorXd bodyAccel(3); // BODY ACCEL
    // bodyAccel <<((data->LINEAR_ACCEL[0])*(1-State::calcAccelSystematicError())), //this is our body accel corrected with systematic error
    //             ((data->LINEAR_ACCEL[1])*(1-State::calcAccelSystematicError())), //this is our body accel corrected with systematic error 
    //             ((data->LINEAR_ACCEL[2])*(1-State::calcAccelSystematicError())); //this is our body accel corrected with systematic error

    bodyAccel <<(data->LINEAR_ACCEL[0]), //this is our body accel WITHOUT systematic error correction
                (data->LINEAR_ACCEL[1]), //this is our body accel WITHOUT systematic error correction
                (data->LINEAR_ACCEL[2]); //this is our body accel WITHOUT systematic error correction

    // Serial.println("Raw:");
    // Serial.println(data->LINEAR_ACCEL[0]);
    // Serial.println(data->LINEAR_ACCEL[1]);
    // Serial.println(data->LINEAR_ACCEL[2]);
    // Serial.println("Correction:");
    // Serial.println(1-State::calcAccelSystematicError());

    theta_k1 = theta+constants::dt*theta_dot;  //This is the standar DCM frame transformation for body to earth using e = Tb
    phi_k1 = phi+constants::dt*phi_dot;        //This is the standar DCM frame transformation for body to earth using e = Tb
    psi_k1 = psi+constants::dt*psi_dot;        //This is the standar DCM frame transformation for body to earth using e = Tb

    // Serial.print(theta_k1);
    // Serial.print(" ");
    // Serial.print(phi_k1);
    // Serial.print(" ");
    // Serial.println(psi_k1);

    Eigen::Matrix3d DCM;
    DCM << 
    cos(theta_k1)*cos(psi_k1), sin(phi_k1)*sin(theta_k1)*cos(psi_k1)-cos(phi_k1)*sin(psi_k1), cos(phi_k1)*sin(theta_k1)*cos(psi_k1)-sin(phi_k1)*sin(psi_k1),
    cos(theta_k1)*sin(psi_k1), sin(phi_k1)*sin(theta_k1)*sin(psi_k1)-cos(phi_k1)*cos(psi_k1), cos(phi_k1)*sin(theta_k1)*sin(psi_k1)-sin(phi_k1)*cos(psi_k1),
    sin(theta_k1 - PI)       , sin(phi_k1)*cos(theta_k1)                                    , cos(phi_k1)*cos(theta_k1);

    // State::print_mtxd(DCM);

    Eigen::VectorXd earthAccel(3);
    // earthAccel << DCM*bodyAccel;

    // BYPASS DCM to isolate accelerometer data
    earthAccel << bodyAccel;

    // State::print_mtxd(bodyAccel);

    matrices::y << 
                0,
                0,
                0,
                0,
                0,
                0,
                earthAccel.coeff(0), //EARTH frame acceleration in x
                earthAccel.coeff(1), //EARTH frame acceleration in y 
                earthAccel.coeff(2), //EARTH frame acceleration in z
                0,
                0,
                0,
                phi_dot,   //see above, but yeah this is our omega in x
                theta_dot, //see above, but yeah this is our omega in y
                psi_dot,   //see above, but yeah this is our omega in z
                0,
                0,
                0,
                data->MAG[0],
                data->MAG[1],
                data->MAG[2];
     // State::print_mtxd(matrices::y);
}


void State::predict()
{
    /*******************************************/
    /*           PREDICT FULL STATE            */
    /*******************************************/

    /*This function takes our previous state x_k_1 and extrapolates it forward in time
    by dt using A which is our state transition matrix yeilding x_kp which is our state prediction
    without measurment correction*/
    matrices::x_kp = matrices::A * matrices::x_k_1;
}


void State::calculateKalmanGain()
{
    /*******************************************/
    /*         CALCULATE KALMAN GAIN           */
    /*******************************************/

    matrices::K = matrices::P_kp*(matrices::H.transpose())*(matrices::H*(matrices::P_kp+matrices::R)*(matrices::H.transpose())).inverse();
    // State::print_mtxd(matrices::K);
}


void State::processCovarianceMatrix()
{
    /*******************************************/
    /*      PREDICT PROCESS COVIARIANCE        */
    /*******************************************/

    // Process covariance noise
    Eigen::MatrixXd randM = Eigen::MatrixXd::Random(21,21)*0.3;
    randM = (randM + Eigen::MatrixXd::Constant(21,21,1))*0.15;

    // Predict process covariance with previous matrix and process noise
    matrices::P_kp = (matrices::A * matrices::P_k_1 * matrices::A.transpose()) + randM;
}


void State::stateDetermination()
{
    /*******************************************/
    /*           DETERMINE FULL STATE          */
    /*******************************************/

    // Determine state matrix based on:
    // previous state, Kalman gain, state prediction, and measurement
    matrices::x_k = matrices::x_kp+matrices::K*matrices::H*(matrices::y-matrices::x_kp);
}


void State::updatePreviousState()
{
    /*******************************************/
    /*           DEFINE PREVIOUS STATE         */
    /*******************************************/
    matrices::x_k_1 = matrices::x_k;
}


void State::updateProcessCovarianceMatrix()
{
    /*******************************************/
    /* PROCESS COVARIANCE AT PREVIOUS TIME STEP*/
    /*******************************************/

    matrices::P_k_1 = (matrices::I-(matrices::K*matrices::H))*matrices::P_kp;
}


void State::updateDynamics()
{
    State::dataAq(State::data);             // ACQUIRE DATA
    State::predict();                       // PREDICT STATE
    State::processCovarianceMatrix();       // PREDICT PROCESS COVARIANCE
    State::calculateKalmanGain();           // CALCULATE KALMAN GAIN
    State::stateDetermination();            // DETERMINE CURRENT STATE
    State::print_mtxd(matrices::x_k);       // PRINT CURRENT STATE
    State::updatePreviousState();           // DEFINE PREVIOUS STATE
    State::updateProcessCovarianceMatrix(); // DEFINE PROCESS COVARIANCE AT PREVIOUS TIME STEP
}


template<typename T>
void State::print_mtxd(const T& X)  
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
