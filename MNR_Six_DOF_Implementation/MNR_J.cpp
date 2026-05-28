/* Auto Generated Jacobian File */

#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include "MNR_J.h"

using namespace std;
using namespace Eigen;

MNR_J::MNR_J() {}

	/* The Main MNR Jacobian Matrix Function */
Eigen::MatrixXd MNR_J::MNR_Jacobian(VectorXd Theta) {

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

	/* Jacobian Matrix */
	MatrixXd J(12, 6);

	J(0, 0) = c1*c4*s6 + c1*c5*c6*s4 + c2*c3*s1*s4*s6 + c2*c6*s1*s3*s5 + c3*c6*s1*s2*s5 - 1.0*s1*s2*s3*s4*s6 - 1.0*c2*c3*c4*c5*c6*s1 + c4*c5*c6*s1*s2*s3;
	J(0, 1) = 1.0*c1*(c2*s3*s4*s6 + c3*s2*s4*s6 + c6*s2*s3*s5 - c2*c3*c6*s5 - c2*c4*c5*c6*s3 - c3*c4*c5*c6*s2);
	J(0, 2) = 1.0*c1*(c2*s3*s4*s6 + c3*s2*s4*s6 + c6*s2*s3*s5 - c2*c3*c6*s5 - c2*c4*c5*c6*s3 - c3*c4*c5*c6*s2);
	J(0, 3) = c4*c5*c6*s1 - s1*s4*s6 - c1*c2*c3*c4*s6 + c1*c4*s2*s3*s6 - c1*c2*c3*c5*c6*s4 + c1*c5*c6*s2*s3*s4;
	J(0, 4) = -c6*(s1*s4*s5 + c1*c2*c5*s3 + c1*c3*c5*s2 + c1*c2*c3*c4*s5 - 1.0*c1*c4*s2*s3*s5);
	J(0, 5) = c4*c6*s1 - c5*s1*s4*s6 - c1*c2*c3*c6*s4 + c1*c6*s2*s3*s4 + c1*c2*s3*s5*s6 + c1*c3*s2*s5*s6 - c1*c2*c3*c4*c5*s6 + c1*c4*c5*s2*s3*s6;

	J(1, 0) = c1*c4*c6 - 1.0*c1*c5*s4*s6 + c2*c3*c6*s1*s4 - 1.0*c6*s1*s2*s3*s4 - 1.0*c2*s1*s3*s5*s6 - 1.0*c3*s1*s2*s5*s6 + c2*c3*c4*c5*s1*s6 - 1.0*c4*c5*s1*s2*s3*s6;
	J(1, 1) = c1*(c2*c6*s3*s4 + c3*c6*s2*s4 + c2*c3*s5*s6 - 1.0*s2*s3*s5*s6 + c2*c4*c5*s3*s6 + c3*c4*c5*s2*s6);
	J(1, 2) = c1*(c2*c6*s3*s4 + c3*c6*s2*s4 + c2*c3*s5*s6 - 1.0*s2*s3*s5*s6 + c2*c4*c5*s3*s6 + c3*c4*c5*s2*s6);
	J(1, 3) = c1*c4*c6*s2*s3 - c4*c5*s1*s6 - c1*c2*c3*c4*c6 - c6*s1*s4 + c1*c2*c3*c5*s4*s6 - c1*c5*s2*s3*s4*s6;
	J(1, 4) = s6*(s1*s4*s5 + c1*c2*c5*s3 + c1*c3*c5*s2 + c1*c2*c3*c4*s5 - 1.0*c1*c4*s2*s3*s5);
	J(1, 5) = c1*c2*c3*s4*s6 - c5*c6*s1*s4 - c4*s1*s6 + c1*c2*c6*s3*s5 + c1*c3*c6*s2*s5 - c1*s2*s3*s4*s6 - c1*c2*c3*c4*c5*c6 + c1*c4*c5*c6*s2*s3;

	J(2, 0) = c1*s4*s5 - 1.0*c2*c5*s1*s3 - 1.0*c3*c5*s1*s2 - 1.0*c2*c3*c4*s1*s5 + c4*s1*s2*s3*s5;
	J(2, 1) = -1.0*c1*(c5*s2*s3 - c2*c3*c5 + c2*c4*s3*s5 + c3*c4*s2*s5);
	J(2, 2) = -1.0*c1*(c5*s2*s3 - c2*c3*c5 + c2*c4*s3*s5 + c3*c4*s2*s5);
	J(2, 3) = 1.0*s5*(c4*s1 + c1*s2*s3*s4 - c1*c2*c3*s4);
	J(2, 4) = c5*s1*s4 - 1.0*c1*c2*s3*s5 - 1.0*c1*c3*s2*s5 + c1*c2*c3*c4*c5 - 1.0*c1*c4*c5*s2*s3;
	J(2, 5) = 0;

	J(3, 0) = c4*s1*s6 + c5*c6*s1*s4 - 1.0*c1*c2*c3*s4*s6 - 1.0*c1*c2*c6*s3*s5 - 1.0*c1*c3*c6*s2*s5 + c1*s2*s3*s4*s6 + c1*c2*c3*c4*c5*c6 - 1.0*c1*c4*c5*c6*s2*s3;
	J(3, 1) = 1.0*s1*(c2*s3*s4*s6 + c3*s2*s4*s6 + c6*s2*s3*s5 - c2*c3*c6*s5 - c2*c4*c5*c6*s3 - c3*c4*c5*c6*s2);
	J(3, 2) = 1.0*s1*(c2*s3*s4*s6 + c3*s2*s4*s6 + c6*s2*s3*s5 - c2*c3*c6*s5 - c2*c4*c5*c6*s3 - c3*c4*c5*c6*s2);
	J(3, 3) = c1*s4*s6 - c1*c4*c5*c6 - c2*c3*c4*s1*s6 + c4*s1*s2*s3*s6 - c2*c3*c5*c6*s1*s4 + c5*c6*s1*s2*s3*s4;
	J(3, 4) = -1.0*c6*(c2*c5*s1*s3 - c1*s4*s5 + c3*c5*s1*s2 + c2*c3*c4*s1*s5 - c4*s1*s2*s3*s5);
	J(3, 5) = c1*c5*s4*s6 - c1*c4*c6 - c2*c3*c6*s1*s4 + c6*s1*s2*s3*s4 + c2*s1*s3*s5*s6 + c3*s1*s2*s5*s6 - c2*c3*c4*c5*s1*s6 + c4*c5*s1*s2*s3*s6;

	J(4, 0) = c4*c6*s1 - c5*s1*s4*s6 - c1*c2*c3*c6*s4 + c1*c6*s2*s3*s4 + c1*c2*s3*s5*s6 + c1*c3*s2*s5*s6 - c1*c2*c3*c4*c5*s6 + c1*c4*c5*s2*s3*s6;
	J(4, 1) = s1*(c2*c6*s3*s4 + c3*c6*s2*s4 + c2*c3*s5*s6 - 1.0*s2*s3*s5*s6 + c2*c4*c5*s3*s6 + c3*c4*c5*s2*s6);
	J(4, 2) = s1*(c2*c6*s3*s4 + c3*c6*s2*s4 + c2*c3*s5*s6 - 1.0*s2*s3*s5*s6 + c2*c4*c5*s3*s6 + c3*c4*c5*s2*s6);
	J(4, 3) = c1*c6*s4 + c1*c4*c5*s6 - 1.0*c2*c3*c4*c6*s1 + c4*c6*s1*s2*s3 - 1.0*c5*s1*s2*s3*s4*s6 + c2*c3*c5*s1*s4*s6;
	J(4, 4) = 1.0*s6*(c2*c5*s1*s3 - c1*s4*s5 + c3*c5*s1*s2 + c2*c3*c4*s1*s5 - c4*s1*s2*s3*s5);
	J(4, 5) = c1*c4*s6 + c1*c5*c6*s4 + c2*c3*s1*s4*s6 + c2*c6*s1*s3*s5 + c3*c6*s1*s2*s5 - 1.0*s1*s2*s3*s4*s6 - 1.0*c2*c3*c4*c5*c6*s1 + c4*c5*c6*s1*s2*s3;

	J(5, 0) = s1*s4*s5 + c1*c2*c5*s3 + c1*c3*c5*s2 + c1*c2*c3*c4*s5 - 1.0*c1*c4*s2*s3*s5;
	J(5, 1) = -1.0*s1*(c5*s2*s3 - c2*c3*c5 + c2*c4*s3*s5 + c3*c4*s2*s5);
	J(5, 2) = -1.0*s1*(c5*s2*s3 - c2*c3*c5 + c2*c4*s3*s5 + c3*c4*s2*s5);
	J(5, 3) = -s5*(c1*c4 + c2*c3*s1*s4 - 1.0*s1*s2*s3*s4);
	J(5, 4) = c2*c3*c4*c5*s1 - c2*s1*s3*s5 - c3*s1*s2*s5 - c1*c5*s4 - c4*c5*s1*s2*s3;
	J(5, 5) = 0;

	J(6, 0) = 0;
	J(6, 1) = s2*s3*s4*s6 - 1.0*c2*c6*s3*s5 - 1.0*c3*c6*s2*s5 - 1.0*c2*c3*s4*s6 + c2*c3*c4*c5*c6 - 1.0*c4*c5*c6*s2*s3;
	J(6, 2) = s2*s3*s4*s6 - 1.0*c2*c6*s3*s5 - 1.0*c3*c6*s2*s5 - 1.0*c2*c3*s4*s6 + c2*c3*c4*c5*c6 - 1.0*c4*c5*c6*s2*s3;
	J(6, 3) = -(c4*s6 + c5*c6*s4)*(c2*s3 + c3*s2);
	J(6, 4) = -1.0*c6*(c5*s2*s3 - c2*c3*c5 + c2*c4*s3*s5 + c3*c4*s2*s5);
	J(6, 5) = s2*s3*s5*s6 - c3*c6*s2*s4 - c2*c3*s5*s6 - c2*c6*s3*s4 - c2*c4*c5*s3*s6 - c3*c4*c5*s2*s6;

	J(7, 0) = 0;
	J(7, 1) = c6*s2*s3*s4 + c2*s3*s5*s6 + c3*s2*s5*s6 - c2*c3*c6*s4 - c2*c3*c4*c5*s6 + c4*c5*s2*s3*s6;
	J(7, 2) = c6*s2*s3*s4 + c2*s3*s5*s6 + c3*s2*s5*s6 - c2*c3*c6*s4 - c2*c3*c4*c5*s6 + c4*c5*s2*s3*s6;
	J(7, 3) = -(c4*c6 - 1.0*c5*s4*s6)*(c2*s3 + c3*s2);
	J(7, 4) = 1.0*s6*(c5*s2*s3 - c2*c3*c5 + c2*c4*s3*s5 + c3*c4*s2*s5);
	J(7, 5) = c2*s3*s4*s6 + c3*s2*s4*s6 + c6*s2*s3*s5 - c2*c3*c6*s5 - c2*c4*c5*c6*s3 - c3*c4*c5*c6*s2;

	J(8, 0) = 0;
	J(8, 1) = c2*c5*s3 + c3*c5*s2 - 1.0*c4*s2*s3*s5 + c2*c3*c4*s5;
	J(8, 2) = c2*c5*s3 + c3*c5*s2 - 1.0*c4*s2*s3*s5 + c2*c3*c4*s5;
	J(8, 3) = -s4*s5*(c2*s3 + c3*s2);
	J(8, 4) = c2*c3*s5 - 1.0*s2*s3*s5 + c2*c4*c5*s3 + c3*c4*c5*s2;
	J(8, 5) = 0;

	J(9, 0) = L4*c1*s4*s5 - L3*c2*s1*s3 - L3*c3*s1*s2 - L2*c2*s1 - L4*c2*c5*s1*s3 - L4*c3*c5*s1*s2 - L4*c2*c3*c4*s1*s5 + L4*c4*s1*s2*s3*s5;
	J(9, 1) = -c1*(L2*s2 - 1.0*L3*c2*c3 + L3*s2*s3 - 1.0*L4*c2*c3*c5 + L4*c5*s2*s3 + L4*c2*c4*s3*s5 + L4*c3*c4*s2*s5);
	J(9, 2) = -1.0*c1*(L3*s2*s3 - L3*c2*c3 - L4*c2*c3*c5 + L4*c5*s2*s3 + L4*c2*c4*s3*s5 + L4*c3*c4*s2*s5);
	J(9, 3) = 1.0*L4*s5*(c4*s1 + c1*s2*s3*s4 - c1*c2*c3*s4);
	J(9, 4) = -1.0*L4*(c1*c2*s3*s5 - c5*s1*s4 + c1*c3*s2*s5 - c1*c2*c3*c4*c5 + c1*c4*c5*s2*s3);
	J(9, 5) = 0;

	J(10, 0) = L2*c1*c2 + L4*s1*s4*s5 + L3*c1*c2*s3 + L3*c1*c3*s2 + L4*c1*c2*c5*s3 + L4*c1*c3*c5*s2 + L4*c1*c2*c3*c4*s5 - 1.0*L4*c1*c4*s2*s3*s5;
	J(10, 1) = -s1*(L2*s2 - 1.0*L3*c2*c3 + L3*s2*s3 - 1.0*L4*c2*c3*c5 + L4*c5*s2*s3 + L4*c2*c4*s3*s5 + L4*c3*c4*s2*s5);
	J(10, 2) = -1.0*s1*(L3*s2*s3 - L3*c2*c3 - L4*c2*c3*c5 + L4*c5*s2*s3 + L4*c2*c4*s3*s5 + L4*c3*c4*s2*s5);
	J(10, 3) = -L4*s5*(c1*c4 + c2*c3*s1*s4 - 1.0*s1*s2*s3*s4);
	J(10, 4) = -L4*(c1*c5*s4 + c2*s1*s3*s5 + c3*s1*s2*s5 - 1.0*c2*c3*c4*c5*s1 + c4*c5*s1*s2*s3);
	J(10, 5) = 0;

	J(11, 0) = 0;
	J(11, 1) = L2*c2 + L3*c2*s3 + L3*c3*s2 + L4*c2*c5*s3 + L4*c3*c5*s2 + L4*c2*c3*c4*s5 - 1.0*L4*c4*s2*s3*s5;
	J(11, 2) = L3*c2*s3 + L3*c3*s2 + L4*c2*c5*s3 + L4*c3*c5*s2 + L4*c2*c3*c4*s5 - 1.0*L4*c4*s2*s3*s5;
	J(11, 3) = -L4*s4*s5*(c2*s3 + c3*s2);
	J(11, 4) = L4*(c2*c3*s5 - 1.0*s2*s3*s5 + c2*c4*c5*s3 + c3*c4*c5*s2);
	J(11, 5) = 0;

	return J;

}
