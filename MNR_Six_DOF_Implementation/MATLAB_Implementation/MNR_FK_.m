function T = MNR_FK_(Theta)

L1 = 0.692;
L2 = 0.65;
L3 = 0.65;
L4 = 0.4545;

t1 = Theta(1);
t2 = Theta(2);
t3 = Theta(3);
t4 = Theta(4);
t5 = Theta(5);
t6 = Theta(6);

c1 = cos(t1);
c2 = cos(t2);
c3 = cos(t3);
c4 = cos(t4);
c5 = cos(t5);
c6 = cos(t6);

s1 = sin(t1);
s2 = sin(t2);
s3 = sin(t3);
s4 = sin(t4);
s5 = sin(t5);
s6 = sin(t6);

% Forward Kinematics

T0101 = c4*s1*s6 + c5*c6*s1*s4 - 1.0*c1*c2*c3*s4*s6 - 1.0*c1*c2*c6*s3*s5 - 1.0*c1*c3*c6*s2*s5 + c1*s2*s3*s4*s6 + c1*c2*c3*c4*c5*c6 - 1.0*c1*c4*c5*c6*s2*s3;
T0102 = c4*c6*s1 - c5*s1*s4*s6 - c1*c2*c3*c6*s4 + c1*c6*s2*s3*s4 + c1*c2*s3*s5*s6 + c1*c3*s2*s5*s6 - c1*c2*c3*c4*c5*s6 + c1*c4*c5*s2*s3*s6;
T0103 = s1*s4*s5 + c1*c2*c5*s3 + c1*c3*c5*s2 + c1*c2*c3*c4*s5 - 1.0*c1*c4*s2*s3*s5;
T0104 = L2*c1*c2 + L4*s1*s4*s5 + L3*c1*c2*s3 + L3*c1*c3*s2 + L4*c1*c2*c5*s3 + L4*c1*c3*c5*s2 + L4*c1*c2*c3*c4*s5 - 1.0*L4*c1*c4*s2*s3*s5;
T0201 = s1*s2*s3*s4*s6 - c1*c5*c6*s4 - c2*c3*s1*s4*s6 - c2*c6*s1*s3*s5 - c3*c6*s1*s2*s5 - c1*c4*s6 + c2*c3*c4*c5*c6*s1 - c4*c5*c6*s1*s2*s3;
T0202 = c1*c5*s4*s6 - c1*c4*c6 - c2*c3*c6*s1*s4 + c6*s1*s2*s3*s4 + c2*s1*s3*s5*s6 + c3*s1*s2*s5*s6 - c2*c3*c4*c5*s1*s6 + c4*c5*s1*s2*s3*s6;
T0203 = c2*c5*s1*s3 - c1*s4*s5 + c3*c5*s1*s2 + c2*c3*c4*s1*s5 - c4*s1*s2*s3*s5;
T0204 = L2*c2*s1 + L3*c2*s1*s3 + L3*c3*s1*s2 - 1.0*L4*c1*s4*s5 + L4*c2*c5*s1*s3 + L4*c3*c5*s1*s2 + L4*c2*c3*c4*s1*s5 - 1.0*L4*c4*s1*s2*s3*s5;
T0301 = c2*c3*c6*s5 - 1.0*c3*s2*s4*s6 - 1.0*c6*s2*s3*s5 - 1.0*c2*s3*s4*s6 + c2*c4*c5*c6*s3 + c3*c4*c5*c6*s2;
T0302 = s2*s3*s5*s6 - c3*c6*s2*s4 - c2*c3*s5*s6 - c2*c6*s3*s4 - c2*c4*c5*s3*s6 - c3*c4*c5*s2*s6;
T0303 = c5*s2*s3 - c2*c3*c5 + c2*c4*s3*s5 + c3*c4*s2*s5;
T0304 = L1 + L2*s2 - 1.0*L3*c2*c3 + L3*s2*s3 - 1.0*L4*c2*c3*c5 + L4*c5*s2*s3 + L4*c2*c4*s3*s5 + L4*c3*c4*s2*s5;
T0401 = 0;
T0402 = 0;
T0403 = 0;
T0404 = 1.0;


T = [ ...
T0101, T0102, T0103, T0104; ...
T0201, T0202, T0203, T0204; ...
T0301, T0302, T0303, T0304; ...
T0401, T0402, T0403, T0404];

end
