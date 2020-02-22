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

void setup()
{
    Eigen::MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
}

void loop()
{
    
}