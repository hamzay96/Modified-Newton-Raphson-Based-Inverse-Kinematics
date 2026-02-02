# Modified Newton–Raphson (MNR) Based Inverse Kinematics – Example Implementation

This repository provides a simple and self-contained MATLAB example demonstrating how to:
- Compute the modified Jacobian matrix Jn (Eq. 14) proposed in the associated paper, and
- Implement the core Modified Newton–Raphson (MNR) inverse kinematics algorithm using this Jacobian.
The purpose of this repository is educational and illustrative: it is intended to help readers understand the formulation and practical implementation of the proposed IK method.

# Overview

- The modified Jacobian Jn is constructed using end-effector frame alignment, where orientation is represented directly by rotation matrix elements instead of Euler angles. The Jacobian is derived symbolically offline and then copied as-is into a numerical MATLAB function for real-time evaluation.
- Using this Jacobian, a basic Newton–Raphson inverse kinematics solver is implemented to compute joint configurations that achieve a desired end-effector pose.

# Example Scope
This repository includes:
- A symbolic workflow to compute the modified Jacobian Jn
- A numerical Jacobian function obtained directly from the symbolic expressions
- A basic MNR-based inverse kinematics implementation
- A single-point IK example
- A linear end-effector Cartesian trajectory example

The implementation is designed as a reference example rather than a fully optimized or hardware-ready solution.

# Order of the Files to Run
1. Calculate_MNR_J.m
	Used to symbolically derive the modified Jacobian Jn for the robot.
	Running this file generates a text file (MNR_Jn.txt) containing the symbolic expressions of Jn.
2. MNR_Jn.m
	A numerical MATLAB function that evaluates Jn.
	Copy the expressions from MNR_Jn.txt into this file.
	This function takes joint angles as input and outputs the modified Jacobian.
3. Six_DOF_FK.m
   Forward kinematics function of the robot. Users must replace this file with their own robot’s FK if a different structure is used.
4. Single_Point_IK.m and IK_with_EEF_Trajectory.m
   Demonstrates MNR-based inverse kinematics along a linear end-effector trajectory.
   
# Limitations of the Current Example
The complete paper presents additional features such as joint limit avoidance, weighted least-norm solutions, clamping strategies, and task prioritization. These components are not included in the current example in order to keep the implementation simple and focused on the inverse kinematics formulation.

All advanced features can be implemented by extending this code, as the paper provides detailed explanations and formulations for each component.

# Intended Use
This example can be adapted for different robot manipulators by modifying the forward kinematics and Jacobian derivation. Users are encouraged to tailor the trajectory generation and solver parameters based on their specific robot configuration and task requirements.

# Citation and Usage
If this code, or any modified version of it, is used in academic publications, the original paper must be properly cited.
The full paper can be viewed here:
DOI: [Insert DOI here]

# Final Note
This repository is intended to complement the paper by providing a clear and reproducible reference implementation. Readers are strongly encouraged to consult the paper for theoretical background, algorithmic details, and advanced implementations beyond the basic inverse kinematics formulation.
