clc
clear variables
close all


syms o1 o2 o3 o4 o5 o6 o7 real
theta = [o1 o2 o3 o4 o5 o6 o7];

% Transformation matrix between consecutive joints
T01 = FK_J(theta,1);
T02 = FK_J(theta,2);
T03 = FK_J(theta,3);
T04 = FK_J(theta,4);
T05 = FK_J(theta,5);
T06 = FK_J(theta,6);
T07 = FK_J(theta,7);
P = T07(1:3,4);

% Z's for each transformation matrix
z0 = [0,0,1]';
z1 = T01(1:3,3);
z2 = T02(1:3,3);
z3 = T03(1:3,3);
z4 = T04(1:3,3);
z5 = T05(1:3,3);
z6 = T06(1:3,3);
z7 = T07(1:3,3);

J = sym('J',[6,7]);

% Linear velocity jacobian
J(1:3,:) = [diff(P,o1),diff(P,o2),diff(P,o3),diff(P,o4),diff(P,o5),diff(P,o6),diff(P,o7)];

% Angular velocity jacobian
J(4:6,:) = [z0 z1 z2 z3 z4 z5 z6];

simplify(J)

% Testing result
theta = [0 90 0 0 0 0 0];
J = subs(J,[o1 o2 o3 o4 o5 o6 o7],theta);

thetaDot = [10 0 0 0 0 0 0]';
V = J*thetaDot;