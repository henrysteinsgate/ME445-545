clc;
clear variables;
close all;

syms c1 s1 c2 s2 c3 s3 l1 l2 l3

T10 = [c1 0 s1 0;
       s1 0 -c1 0;
       0 1 0 0.175
       0 0 0 1];
T21 = [c2 -s2 0 0.475*c2;
       s2 c2 0 0.475*s2;
       0 0 1 0;
       0 0 0 1];
T32 = [c3 -s3 0 0.325*c3;
       s3 c3 0 0.325*s3;
       0 0 1 0;
       0 0 0 1];
T20 = T10*T21;
T30 = T20*T32;

P0G1_0 = [0;0;l1];
P1G2_1 = [l2*c2;l2*s2;0];
P2G3_2 = [l3*c3;l3*s3;0];

P0G2_0 = T10*[P1G2_1;1];
P0G3_0 = T20*[P2G3_2;1];
