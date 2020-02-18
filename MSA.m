clc
clear all
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
A2 = [ Lambda_r                   -Lambda_r    zero_6                                          
       Lambda_e*K11_1-Kq*Lambda_e Kq*Lambda_e  Lambda_e*K11_2-Kq*Lambda_e   
       K11_1                      K12_1        zero_6                                         
       K12_1                      K22_1        K11_2                     ]
   
B2 = [zero_6
      Lambda_e
      zero_6
      K12_2]
  
C2 = [zero_6 zero_6 K21_2]
D2 = K22_2
%Kc = D2-C2*A2*B2'
Kc = A2'*B2
Kc = C2*Kc
Kc = inv(D2 - Kc)
% [0     [A2 B2    [dt1
%  W3] =  C2 D2] *  dt2
%                   dt3]
%W3 = Kc*dt
% W3 = [100 0 0 0 0 0]'
for i = 1:10
    for j = 1:10
        for k = 1:10
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