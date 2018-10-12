clc;
clear variables;
close all;

data = xlsread('two_dof_arm_properties'); % Load data

syms o1 o2 real
theta = [o1 o2];


%%

Mass = data(1:2,1).'; % Load the Mass data (Unit in kg)
Mass_location = data(1:2,10:12); % Load the Center of Mass location represented in URDF joint frames
%

%% 
% Loads the inertia matrix of each joint. Inertia matrix is represented in the inertia frames
Ixx = data(1:2,2).'; 
Ixy = data(1:2,3).';
Ixz = data(1:2,4).';
Iyy = data(1:2,5).';
Iyz = data(1:2,6).';
Izz = data(1:2,7).';
%

%% Set up inertia matrixs
IG_original = zeros(3,3,2);
IG = sym(zeros(3,3,2));
for i = 1:2
   IG_original(:,:,i)=[Ixx(i) -Ixy(i) -Ixz(i);
       -Ixy(i) Iyy(i) -Iyz(i);
       -Ixz(i) -Iyz(i) Izz(i)];
   M_location(1:3,i) = [Mass_location(i,1) Mass_location(i,2) Mass_location(i,3)].';
end
%

%% Calculate the trasfermation matrixs for each joint to the base frame. (in DH frames) 
T0n(1:4,1:4,1) = FK_J(theta,1);
T0n(1:4,1:4,2) = FK_J(theta,2);
    

%% Z axis for the DH frames

z = sym(zeros(3,3));
z(:,1) = [0,0,1].';  % z0
z(:,2) = T0n(1:3,3,1);



%% Calculate the Position vectors from the base joint to each link.'s center of mass
P0G = sym(zeros(4,2));

P0G(:,1) = [M_location(:,1);1];
P0G(:,2) = T0n(:,:,1)*[M_location(2,2)*cos(o2);M_location(2,2)*sin(o2);0;1];


P0G(4,:) = [];

%% Transform inertia matrix from inertia frame to DH frame

for i=1:2
    IG(:,:,i) =T0n(1:3,1:3,i)*IG_original(:,:,i)*T0n(1:3,1:3,i).'; 
end

%% Calculate jacobian of the robot and mass matrix of the robot.
JvG = sym(zeros(3,2,2));
for i = 1:2
    for k = 1:i
        JvG(1:3,k,i)=diff(P0G(:,i),theta(k)); % direct differentiate method
    end
end

Jw = sym(zeros(3,2,2));
M = sym(zeros(2,2));
for i = 1:2
    Jw(1:3,1:i,i) = z(:,1:i);
    M = M + Mass(i)*JvG(:,:,i)'*JvG(:,:,i) + Jw(:,:,i)'*IG(:,:,i)*Jw(:,:,i);
end

%% Calaulate Centrifugal force matrix C of Sawyer

% calculate m matrixs
m = sym(zeros(2,2,2));
for i = 1:2
    m(:,:,i) = diff(M,theta(i));
end

% calculate b matrixs
b = sym(zeros(2,2,2));
for i = 1:2
    for j = 1:2
        for k = 1:2
            b(i,j,k) = (1/2)*(m(i,j,k) + m(i,k,j) - m(j,k,i));
        end
    end
end

C = sym(zeros(2,2));
for i = 1:2
    for j = 1:2
        C(i,j) = b(i,j,j);
    end
end

%% Calaulate Coriolis forces matrix of Sawyer
B = sym(zeros(2,1));
for i = 1:2
    n = 1;
    for j = 1:1
        for k = j+1:2
            B(i,n) = 2 * b(i, j, k);
            n = n + 1;
        end
    end
end

%% Calculate gravity vector of Sawyer
Gsub1 = sym(zeros(2,6));
Gsub2 = sym(zeros(6,1));

n = 1;
for i = 1:3:6
    Gsub1(:,i:i+2) = JvG(:,:,n).';
    Gsub2(i:i+2) = Mass(n)*[0 0 -9.81].';
    n = n + 1;
end

G = - Gsub1*Gsub2;

%% Convert the calculated symbolic matrixs into function so they can be used in simulink. It generally takes more than 4 hours.

M = simplifyMatrix(M);
B = simplifyMatrix(B);
C = simplifyMatrix(C);
G = simplifyMatrix(G);
% 
% % M = vpa(M,4);
% % B = vpa(B,4);
% % C = vpa(C,4);
% % G = vpa(G,4);
% 
% 
% 
% matlabFunction(M,'File','M_func.m');
% matlabFunction(B,'File','B_func.m');
% matlabFunction(C,'File','C_func.m');
% matlabFunction(G,'File','G_func.m');