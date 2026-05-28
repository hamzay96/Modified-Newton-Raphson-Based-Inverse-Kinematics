/* Auto Generated FK File */

#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include "MNR_FK.h"

using namespace std;
using namespace Eigen;

MNR_FK::MNR_FK() {}

	/* The Main MNR Jacobian Matrix Function */
Eigen::MatrixXd MNR_FK::FK(VectorXd Theta) {

	/* Joint Angles */
	double t1 = Theta(0);
	double t2 = Theta(1);
	double t3 = Theta(2);
	double t4 = Theta(3);
	double t5 = Theta(4);
	double t6 = Theta(5);

	/* Link Lengths */
	double L1 = 0.692;
	double L2 = 0.65;
	double L3 = 0.65;
	double L4 = 0.4545;

	/* Cos and Sin */
	double c1 = cos(t1);
	double c2 = cos(t2);
	double c3 = cos(t3);
	double c4 = cos(t4);
	double c5 = cos(t5);
	double c6 = cos(t6);

	double s1 = sin(t1);
	double s2 = sin(t2);
	double s3 = sin(t3);
	double s4 = sin(t4);
	double s5 = sin(t5);
	double s6 = sin(t6);

	/* Forward Kinematics Transformation Matrix */
	MatrixXd T06 = MatrixXd::Identity(4, 4);

	T06(0, 0) = c4*s1*s6 + c5*c6*s1*s4 - 1.0*c1*c2*c3*s4*s6 - 1.0*c1*c2*c6*s3*s5 - 1.0*c1*c3*c6*s2*s5 + c1*s2*s3*s4*s6 + c1*c2*c3*c4*c5*c6 - 1.0*c1*c4*c5*c6*s2*s3;
	T06(0, 1) = c4*c6*s1 - c5*s1*s4*s6 - c1*c2*c3*c6*s4 + c1*c6*s2*s3*s4 + c1*c2*s3*s5*s6 + c1*c3*s2*s5*s6 - c1*c2*c3*c4*c5*s6 + c1*c4*c5*s2*s3*s6;
	T06(0, 2) = s1*s4*s5 + c1*c2*c5*s3 + c1*c3*c5*s2 + c1*c2*c3*c4*s5 - 1.0*c1*c4*s2*s3*s5;
	T06(0, 3) = L2*c1*c2 + L4*s1*s4*s5 + L3*c1*c2*s3 + L3*c1*c3*s2 + L4*c1*c2*c5*s3 + L4*c1*c3*c5*s2 + L4*c1*c2*c3*c4*s5 - 1.0*L4*c1*c4*s2*s3*s5;

	T06(1, 0) = s1*s2*s3*s4*s6 - c1*c5*c6*s4 - c2*c3*s1*s4*s6 - c2*c6*s1*s3*s5 - c3*c6*s1*s2*s5 - c1*c4*s6 + c2*c3*c4*c5*c6*s1 - c4*c5*c6*s1*s2*s3;
	T06(1, 1) = c1*c5*s4*s6 - c1*c4*c6 - c2*c3*c6*s1*s4 + c6*s1*s2*s3*s4 + c2*s1*s3*s5*s6 + c3*s1*s2*s5*s6 - c2*c3*c4*c5*s1*s6 + c4*c5*s1*s2*s3*s6;
	T06(1, 2) = c2*c5*s1*s3 - c1*s4*s5 + c3*c5*s1*s2 + c2*c3*c4*s1*s5 - c4*s1*s2*s3*s5;
	T06(1, 3) = L2*c2*s1 + L3*c2*s1*s3 + L3*c3*s1*s2 - 1.0*L4*c1*s4*s5 + L4*c2*c5*s1*s3 + L4*c3*c5*s1*s2 + L4*c2*c3*c4*s1*s5 - 1.0*L4*c4*s1*s2*s3*s5;

	T06(2, 0) = c2*c3*c6*s5 - 1.0*c3*s2*s4*s6 - 1.0*c6*s2*s3*s5 - 1.0*c2*s3*s4*s6 + c2*c4*c5*c6*s3 + c3*c4*c5*c6*s2;
	T06(2, 1) = s2*s3*s5*s6 - c3*c6*s2*s4 - c2*c3*s5*s6 - c2*c6*s3*s4 - c2*c4*c5*s3*s6 - c3*c4*c5*s2*s6;
	T06(2, 2) = c5*s2*s3 - c2*c3*c5 + c2*c4*s3*s5 + c3*c4*s2*s5;
	T06(2, 3) = L1 + L2*s2 - 1.0*L3*c2*c3 + L3*s2*s3 - 1.0*L4*c2*c3*c5 + L4*c5*s2*s3 + L4*c2*c4*s3*s5 + L4*c3*c4*s2*s5;

	T06(3, 0) = 0;
	T06(3, 1) = 0;
	T06(3, 2) = 0;
	T06(3, 3) = 1.0;

	return T06;

}
