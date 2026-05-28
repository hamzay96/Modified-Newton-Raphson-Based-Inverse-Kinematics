/*
 * MNR_FK.h
 *
 *  Created on: May 20, 2026
 *      Author: Dr. Hamza Khan
 */

#ifndef KINEMATICS_MNR_FK_H_
#define KINEMATICS_MNR_FK_H_


#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

class MNR_FK {
public:

	MNR_FK();
	/* The Forward Kinematics Function*/
	Eigen::MatrixXd FK(VectorXd Theta);

};


#endif /* KINEMATICS_MNR_FK_H_ */
