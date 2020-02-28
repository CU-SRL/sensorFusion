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
    matrices::H.block<3,3>(0,6) = I_block;
    matrices::H.block<3,3>(3,12) = I_block;
    // matrices::H.block<3,3>(6,18) = I_block;
    
    // State::print_mtxd(matrices::H);
}

// Destructor
State::~State(){}




// Eigen::MatrixXd dcmBodyToEarth(double theta, double phi, double psi){

//     // theta = theta*PI/180;
//     // phi = phi*PI/180;
//     // psi = psi*PI/180;

//     Eigen::MatrixXd DCM(3,3);
//     DCM << 
//     cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)-sin(phi)*sin(psi),
//     cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)-cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi),
//     sin(theta - PI)        , sin(phi)*cos(theta)                           , cos(phi)*cos(theta)                           ;

//     return DCM;
// }




// This is updating our observation vector from out struct of IMU data
    //IMPORTANT--this wont work once we update our sensor suite
void State::dataAq(IMUdata *data){
    /*******************************************/
    /* CONVERT ACCELERATION FROM BODY TO EARTH */
    /*******************************************/
    //Lets check this with how we have the IMU oriented at launch
    //all in rads/s or rads
    
    double p =  (data->GYRO[0])*(1-State::calcGyroSystematicError()) * PI/180;//Roll rate
    double q =  (data->GYRO[1])*(1-State::calcGyroSystematicError()) * PI/180;//Pitch rate
    double r =  (data->GYRO[2])*(1-State::calcGyroSystematicError()) * PI/180;//Yaw rate

    double phi   = matrices::x_k_1.coeff(9);     // pulling these euler angles from the last state matrix 
    double theta = matrices::x_k_1.coeff(10);   // pulling these euler angles from the last state matrix
    double psi   = matrices::x_k_1.coeff(11);   // pulling these euler angles from the last state matrix

    double phi_k1 = 0.0;    //  Phi at K+1
    double theta_k1 = 0.0;  //  Theta at K+1
    double psi_k1 = 0.0;    // PSI at K+1

    // Serial.print(phi);
    // Serial.print(" ");
    // Serial.print(theta);
    // Serial.print(" ");
    // Serial.print(psi);
    // Serial.print(" ");

    double phi_dot    = p + (q*sin(phi) + r*cos(phi)) * tan(theta);  //These are all of our angular rates relative to earth:  phi dot
    double theta_dot  = q*cos(phi)-r*sin(phi);                       //These are all of our angular rates relative to earth:  theta dot
    double psi_dot    = (q*sin(phi)+r*cos(phi))/cos(theta);          //These are all of our angular rates relative to earth:  psi dot

    // Serial.print(phi_dot);
    // Serial.print(" ");
    // Serial.print(theta_dot);
    // Serial.print(" ");
    // Serial.println(psi_dot);

    Eigen::VectorXd bodyAccel(3);
    bodyAccel <<((data->LINEAR_ACCEL[0])*(1-State::calcAccelSystematicError())), //this is our body accel corrected with systematic error
                ((data->LINEAR_ACCEL[1])*(1-State::calcAccelSystematicError())), //this is our body accel corrected with systematic error 
                ((data->LINEAR_ACCEL[2])*(1-State::calcAccelSystematicError())); //this is our body accel corrected with systematic error

    theta_k1 = theta+constants::dt*theta_dot;  //This is the standar DCM frame transformation for body to earth using e = Tb
    phi_k1 = phi+constants::dt*phi_dot;        //This is the standar DCM frame transformation for body to earth using e = Tb
    psi_k1 = psi+constants::dt*psi_dot;        //This is the standar DCM frame transformation for body to earth using e = Tb

    Eigen::Matrix3d DCM;
    DCM << 
    cos(theta_k1)*cos(psi_k1), sin(phi_k1)*sin(theta_k1)*cos(psi_k1)-cos(phi_k1)*sin(psi_k1), cos(phi_k1)*sin(theta_k1)*cos(psi_k1)-sin(phi_k1)*sin(psi_k1),
    cos(theta_k1)*sin(psi_k1), sin(phi_k1)*sin(theta_k1)*sin(psi_k1)-cos(phi_k1)*cos(psi_k1), cos(phi_k1)*sin(theta_k1)*sin(psi_k1)-sin(phi_k1)*cos(psi_k1),
    sin(theta_k1 - PI)       , sin(phi_k1)*cos(theta_k1)                                    , cos(phi_k1)*cos(theta_k1);

    // State::print_mtxd(DCM);

    Eigen::VectorXd earthAccel(3);
    earthAccel << DCM*bodyAccel;

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

    matrices::R = temp*temp.transpose();

    // Serial.println("R");
    // State::print_mtxd(matrices::R);
    // Serial.println("P_kp");
    // State::print_mtxd(matrices::P_kp);
    // Serial.println("H");
    // State::print_mtxd(matrices::H);

    matrices::K = matrices::P_kp*(matrices::H.transpose())*(matrices::H*(matrices::P_kp+matrices::R)*(matrices::H.transpose())).inverse();
    State::print_mtxd(matrices::K);
    State::count++;
    if(count==5)
    {
        while(true)
        {

        }
    }
}

void State::processCovarianceMatrix()
{
    matrices::P_kp = matrices::A * matrices::P_k_1 * matrices::A.transpose();
    // State::print_mtxd(matrices::P_kp);
}

void State::stateDetermination()
{
    matrices::x_k = matrices::x_kp+matrices::K*matrices::H*(matrices::y-matrices::x_kp);
    // Serial.println("K");
    // State::print_mtxd(matrices::K);
    // Serial.println("y");
    // State::print_mtxd(matrices::y);
}

void State::updatePreviousState()
{
    matrices::x_k_1 = matrices::x_k;
}

void State::updateProcessCovarianceMatrix()
{
    matrices::P_k_1 = (matrices::I-(matrices::K*matrices::H))*matrices::P_kp;
}

void State::updateDynamics()
{
    State::dataAq(State::data); // ACQUIRE DATA
    State::predict(); // 
    State::processCovarianceMatrix(); //
    State::calculateKalmanGain(); // CALCULATE KALMAN GAIN
    State::stateDetermination();
    // State::print_mtxd(matrices::x_k);
    State::updatePreviousState();
    State::updateProcessCovarianceMatrix();
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

