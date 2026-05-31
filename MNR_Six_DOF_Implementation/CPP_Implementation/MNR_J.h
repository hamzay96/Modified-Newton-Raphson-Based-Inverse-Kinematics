/*
 * MNR_J.h
 *
 *  Created on: May 20, 2026
 *      Author: Dr. Hamza Khan
 */

#ifndef KINEMATICS_MNR_J_H_
#define KINEMATICS_MNR_J_H_


#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

class MNR_J {
public:

    MNR_J();

    /* The Main Function Called From Other Codes */
    Eigen::MatrixXd MNR_Jacobian(VectorXd Theta);

};


#endif /* KINEMATICS_MNR_J_H_ */
