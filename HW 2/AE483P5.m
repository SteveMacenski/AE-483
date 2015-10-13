% AE 483 P 5

%s = sqrt(3)/2
%t = sqrt(2)/2

%R_01 = [0 s s;-1 0 0;0 -.5 .5]
%R_02 = [-t t 0;-t -t 0;0 0 1]
clc
clear all

syms t1 t2 t3 td1 td2 td3

R_01 = [1 0 0;0 cos(t1) -sin(t1);0 sin(t1) cos(t1)];
R_12 = [cos(t2) -sin(t2) 0; sin(t2) cos(t2) 0; 0 0 1];
R_23 = [cos(t3) 0 sin(t3); 0 1 0; -sin(t3) 0 cos(t3)];
R_03 = R_01*R_12*R_23;

partA = transpose(R_01)*[0 0 0;0 -sin(t1) -cos(t1);0 cos(t1) -sin(t1)];
partA = [0 0 0;0 0 -td1;0 td1 0];
partB = transpose(R_12)*[-sin(t2) -cos(t2) 0; cos(t2) -sin(t2) 0; 0 0 0];
partB = [0 -td2 0;td2 0 0;0 0 0];
partC = transpose(R_23)*[-sin(t3) 0 cos(t3);0 0 0;-cos(t3) 0 -sin(t3)];
partC = [0 0 td3;0 0 0;-td3 0 0];

syms td1 td2 td3
partD1 = transpose(R_23)*transpose(R_12)*[td1;0;0];
partD2 = transpose(R_23)*[0;0;td2]
% partD3 = partD1 + partD2 + [0 0 td3;0 0 0;-td3 0 0];
