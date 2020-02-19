clc
clear all
close all
k0 = 1e6; % Actuator stif
%material and shape parameters
E = 70e9; %Young's modulus
G = 25.5e9; %shear modulus
d = 50e-3;
%for cylinder
S = pi*d^2/4;
Iy = pi*d^4/64;
Iz = pi*d^4/64;
J = Iy + Iz;
L = 10;

K11 = [E*S/L 0                 0                 0           0                 0;
    0           12*E*Iz/L^3  0                 0           0                 6*E*Iy/L^2;
    0           0                  12*E*Iy/L^3 0           -6*E*Iy/L^2 0;
    0           0                  0                 G*J/L 0                 0;
    0           0                  -6*E*Iy/L^2 0           4*E*Iy/L      0;
    0           6*E*Iy/L^2   0                 0           0                 4*E*Iz/L];

K12 = [-E*S/L 0                 0                 0           0                 0;
    0           -12*E*Iz/L^3  0                 0           0                 6*E*Iy/L^2;
    0           0                  -12*E*Iy/L^3 0           -6*E*Iy/L^2 0;
    0           0                  0                 -G*J/L 0                 0;
    0           0                  6*E*Iy/L^2 0           4*E*Iy/L      0;
    0           -6*E*Iy/L^2   0                 0           0                 4*E*Iz/L];

K21 = K12
K22 = [E*S/L 0                 0                 0           0                 0;
    0           12*E*Iz/L^3  0                 0           0                 -6*E*Iy/L^2;
    0           0                  12*E*Iy/L^3 0           6*E*Iy/L^2 0;
    0           0                  0                 G*J/L 0                 0;
    0           0                  6*E*Iy/L^2 0           4*E*Iy/L      0;
    0           -6*E*Iy/L^2   0                 0           0                 4*E*Iz/L];
K11_1 = K11
K11_2 = K11
K12_1 = K12
K12_2 = K12
K21_1 = K21
K21_2 = K21
K22_1 = K22
K22_2 = K22
Kq = K11+K21+K12+K22
% [W1    [K11_1 K12_1 0     0
%  W2  =  K21_1 K22_1 0     0
%  W3     0     0     K11_2 K12_2 
%  W4]    0     0     K21_2 K22_2]
% 
% [0     [ Lambda_r                   -Lambda_r    0                           0                  [dt1
%  0       Lambda_e*K11_1-Kq*Lamdba_e Kq*Lambda_e  Lambda_e*K11_2-Kq*Lamdba_e  Lambda_e*K12_2      dt2
%  0   =   K11_1                      K12_1        0                           0                *  dt3
%  0       K12_1                      K22_1        K11_2                       K12_2               dt4]
%  W4]     0                          0            K21_2                       K22_2]    

Lambda_r = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0; 0 0 0 0 0 1]
Lambda_e = [0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 1 0; 0 0 0 0 0 0] 
zero_6 = zeros(6,6)
zero_3 = zeros(3,3)
for i = 1:10
    for j = 1:10
        for k = 1:10
            ef = [i j k 0 0 0]'
            phi_1_1 = atan2(ef(2), ef(1)) %angle of the first roational joint (first leg)
            phi_2_1 = atan2(ef(3), ef(1)) %angle of the first roational joint (second leg)
            phi_3_1 = atan2(ef(3), ef(1)) %angle of the first roational joint (third leg)

            phi_1_2 = atan2(ef(2)-L*sin(phi_1_1), ef(1)-L*cos(phi_1_1)) %angle of the first roational joint (first leg)
            phi_2_2 = atan2(ef(2)-L*sin(phi_2_1), ef(1)-L*cos(phi_2_1)) %angle of the first roational joint (second leg)
            phi_3_2 = atan2(ef(2)-L*sin(phi_3_1), ef(1)-L*cos(phi_3_1)) %angle of the first roational joint (third leg)
            
            R11 = [cos(phi_1_1) -sin(phi_1_1) 0
                   sin(phi_1_1) cos(phi_1_1)  0
                   0                 0        1]
              
            R12 = [cos(phi_1_2) -sin(phi_1_2) 0
                   sin(phi_1_2) cos(phi_1_2)  0
                   0             0            1]
              
            Q11 = [R11 zero_3
                   zero_3 R11]
              
            Q12 = [R12 zero_3
                  zero_3 R12]
             
            R21 = [cos(phi_2_1) -sin(phi_2_1) 0
                  sin(phi_2_1) cos(phi_2_1)   0
                    0                   0     1]
              
            R22 = [cos(phi_2_2) -sin(phi_2_2) 0 
                  sin(phi_2_2) cos(phi_2_2)   0
                    0               0         1]
              
            Q21 = [R21 zero_3
                  zero_3 R21]
              
            Q22 = [R22 zero_3
                  zero_3 R22]
              
            R31 = [cos(phi_3_1) -sin(phi_3_1) 0
                   sin(phi_3_1) cos(phi_3_1)  0
                    0                   0     1]
              
            R32 = [cos(phi_3_2) -sin(phi_3_2) 0
                   sin(phi_3_2) cos(phi_3_2)  0
                        0           0         1]
              
            Q31 = [R31 zero_3
                  zero_3 R31]
              
            Q32 = [R32 zero_3
                  zero_3 R32]
            %For the first leg
            Link_1 = [Q11*K11*Q11' Q11*K12*Q11' zero_6
                      Q11*K21*Q11' Q11*K22*Q11' zero_6
                      zero_6       Q12*K11*Q12' Q12*K12*Q12'
                      zero_6       Q12*K21*Q12' Q12*K12*Q12']
                  
            %For the second leg
            Link_2 = [Q21*K11*Q21' Q21*K12*Q21' zero_6
                      Q21*K21*Q21' Q21*K22*Q21' zero_6
                      zero_6       Q22*K11*Q22' Q22*K12*Q22'
                      zero_6       Q22*K21*Q22' Q22*K12*Q22']
              
            %For the third leg
            Link_3 = [Q31*K11*Q31' Q31*K12*Q31' zero_6
                      Q31*K21*Q31' Q31*K22*Q31' zero_6
                      zero_6       Q32*K11*Q32' Q32*K12*Q32'
                      zero_6       Q32*K21*Q32' Q32*K12*Q32']
                  
            %-----------------joints------------------------
            Joint_1 = [zero_6        Lambda_e
                       Lambda_e(6,:) (K12*(Lambda_e(6,:))')']
            Joint_2 = Joint_1
            Joint_3 = Joint_1
            %----------------passive joints-----------------
            A = [0 0 0 0 Lambda_r(6,:) -(Lambda_r(6,:)) 0 0
                Lambda_r(6,:) (Lambda_r(6,:)) 0 0 0 0 0 0
                Lambda_e(6,:) 0 0 0 0 0 0 0 0 0 0 0 0
                0 Lambda_e(6,:) 0 0 0 0 0 0 0 0 0 0 0
                0 0 0 0 0 0 (Lambda_r(6,:)) (Lambda_r(6,:))
                0 0 Lambda_r(6,:) Lambda_e(6,:) 0 0 0 0
                0 0 Lambda_e(6,:) 0 0 0 0 0 0 0 0 0 0
                0 0 0 Lambda_e(6,:) 0 0 0 0 0 0 0 0 0]
            [m,n] = size(A)
                       
A2 = A(1:(m-1),1:(n-1))
   
B2 = A(1:(m-1),n)
  
C2 = A(m,1:(n-1))
D2 = 0
% Kc = D2-C2*inv(A2)*B2'
Kc = A2'*B2
Kc = C2*Kc
Kc = D2 - Kc
% [0     [A2 B2    [dt1
%  W3] =  C2 D2] *  dt2
%                   dt3]
%W3 = Kc*dt
% W3 = [100 0 0 0 0 0]'
            W3 = [0 0 0 0 0 0]'
            dt= inv(Kc+eye(6)*1e-1)*W3
            dr=sqrt(dt(1)^2+dt(2)^2+dt(3)^2);
            dt0(i,j,k) = dr
            
            W3 = [100 0 0 0 0 0]'
            dt= inv(Kc+eye(6)*1e-1)*W3
            dr=sqrt(dt(1)^2+dt(2)^2+dt(3)^2);
            dt1(i,j,k) = dr
            
            W3 = [0 100 0 0 0 0]'
            dt= inv(Kc+eye(6)*1e-1)*W3
            dr=sqrt(dt(1)^2+dt(2)^2+dt(3)^2);
            dt2(i,j,k) = dr
            
            W3 = [0 0 100 0 0 0]'
            dt= inv(Kc+eye(6)*1e-1)*W3
            dr=sqrt(dt(1)^2+dt(2)^2+dt(3)^2);
            dt3(i,j,k) = dr
            
            W3 = [100 100 100 0 0 0]'
            dt= inv(Kc+eye(6)*1e-1)*W3
            dr=sqrt(dt(1)^2+dt(2)^2+dt(3)^2);
            dt4(i,j,k) = dr
        end
    end
end

figure()
x = 1:10
y = 1:10
z = 1:10
% subplot(2,2,1)
figure(1)
subplot(1,3,1);
surf(x,y,dt0(:,:,1))
title("deflection for 100N along x for XY")
zx = reshape(dt0(1,:,:), 10, 10)
subplot(1,3,2);
surf(y,z,zx)
title("deflection for 100N along x for YZ")
zy = reshape(dt0(:,1,:), 10, 10)
subplot(1,3,3);
surf(x,z,zy)
title("deflection for 100N along x for XZ")

figure(2)
subplot(1,3,1);
surf(x,y,dt1(:,:,1))
title("deflection for 100N along y for XY")
zx = reshape(dt1(1,:,:), 10, 10)
subplot(1,3,2);
surf(y,z,zx)
title("deflection for 100N along y for YZ")
zy = reshape(dt1(:,1,:), 10, 10)
subplot(1,3,3);
surf(x,z,zy)
title("deflection for 100N along y for XZ")

figure(3)
subplot(1,3,1);
surf(x,y,dt2(:,:,1))
title("deflection for 100N along z for XY")
zx = reshape(dt2(1,:,:), 10, 10)
subplot(1,3,2);
surf(y,z,zx)
title("deflection for 100N along z for YZ")
zy = reshape(dt2(:,1,:), 10, 10)
subplot(1,3,3);
surf(x,z,zy)
title("deflection for 100N along z for XZ")

figure(4)
subplot(1,3,1);
surf(x,y,dt3(:,:,1))
title("deflection without force for XY")
zx = reshape(dt3(1,:,:), 10, 10)
subplot(1,3,2);
surf(y,z,zx)
title("deflection without force for YZ")
zy = reshape(dt3(:,1,:), 10, 10)
subplot(1,3,3);
surf(x,z,zy)
title("deflection without force for XZ")

figure(5)
subplot(1,3,1);
surf(x,y,dt4(:,:,1))
title("deflection with all the forces for XY")
zx = reshape(dt4(1,:,:), 10, 10)
subplot(1,3,2);
surf(y,z,zx)
title("deflection with all the forces for YZ")
zy = reshape(dt4(:,1,:), 10, 10)
subplot(1,3,3);
surf(x,z,zy)
title("deflection with all the forces for XZ")

% subplot(2,2,1)