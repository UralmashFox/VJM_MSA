clc
clear all
%Inverse kinematics:
L = 10; %Length of each leg
%if the reference frame for each leg is located in the translation joint,
%then the kinematics counts for each of 3 legs in the same way:
ef = [100 1 1 0 0 0]' %position of end - effector
for i = 1:10
    for j = 1:10
        for kk = 1:10
            ef = [i j kk 0 0 0]'
            phi_1_1 = atan2(ef(2), ef(1)) %angle of the first roational joint (first leg)
            phi_2_1 = atan2(ef(3), ef(1)) %angle of the first roational joint (second leg)
            phi_3_1 = atan2(ef(3), ef(1)) %angle of the first roational joint (third leg)

            phi_1_2 = atan2(ef(2)-L*sin(phi_1_1), ef(1)-L*cos(phi_1_1)) %angle of the first roational joint (first leg)
            phi_2_2 = atan2(ef(2)-L*sin(phi_2_1), ef(1)-L*cos(phi_2_1)) %angle of the first roational joint (second leg)
            phi_3_2 = atan2(ef(2)-L*sin(phi_3_1), ef(1)-L*cos(phi_3_1)) %angle of the first roational joint (third leg)


            %For the first leg 
            T_1_1 = [0 0 ef(3) 0 0 phi_1_1] %position of the first joint now

            T11 = [cos(phi_1_1) -sin(phi_1_1) 0 0
                   sin(phi_1_1)  cos(phi_1_1) 0 0
                        0            0        1 ef(3)
                        0            0        0 1    ]

            T_1_2 = [L*cos(phi_1_1) L*sin(phi_1_1) ef(3) 0 0 phi_1_2]%position of the second joint now

            T12 = [cos(deg2rad(90)+phi_1_2) -sin(deg2rad(90)+phi_1_2) 0 0
                   sin(deg2rad(90)+phi_1_2)  cos(deg2rad(90)+phi_1_2) 0 0
                        0                                    0        1 ef(3)
                        0                                    0        0 1    ]

            %For the second leg 
            T_2_1 = [0 0 ef(3) 0 0 phi_2_1] %position of the first joint now

            T21 = [cos(phi_2_1) -sin(phi_2_1) 0 0
                   sin(phi_2_1)  cos(phi_2_1) 0 0
                        0            0        1 ef(2)
                        0            0        0 1    ]

            T_2_2 = [L*cos(phi_2_1) L*sin(phi_2_1) ef(3) 0 0 phi_2_2]%position of the second joint now

            T22 = [cos(deg2rad(90)+phi_1_2) -sin(deg2rad(90)+phi_1_2) 0 0
                   sin(deg2rad(90)+phi_1_2)  cos(deg2rad(90)+phi_1_2) 0 0
                        0            0        1 ef(2)
                        0            0        0 1    ]

            %For the third leg 
            T_3_1 = [0 0 ef(3) 0 0 phi_3_1] %position of the first joint now

            T31 = [cos(phi_3_1) -sin(phi_3_1) 0 0
                   sin(phi_3_1)  cos(phi_3_1) 0 0
                        0            0        1 ef(1)
                        0            0        0 1    ]

            T_3_2 = [L*cos(phi_3_1) L*sin(phi_3_1) ef(3) 0 0 phi_3_2]%position of the second joint now

            T32 = [cos(deg2rad(90)+phi_3_2) -sin(deg2rad(90)+phi_3_2) 0 0
                   sin(deg2rad(90)+phi_3_2)  cos(deg2rad(90)+phi_3_2) 0 0
                        0            0        1 ef(1)
                        0            0        0 1    ]

            %now result in the world frame:
            T_1_w = [ 1 0 0 0
                      0 0 1 ef(2)
                      0 -1 0 0
                      0 0 0 1] %transformation martix from the first leg to world frame

            T_2_w = [ 1 0 0 0
                      0 1 1 1
                      0 0 1 ef(3)
                      0 0 0 1] %transformation martix from the second leg to world frame

            T_3_w = [0 0 -1 ef(1)
                     0 -1 0 0
                     -1 0 0 0
                     0 0 0 1] %transformation martix from the third leg to world frame
            %Total transformation from legs to world:
            %for first leg
            T11 = T_1_w*T11 %from base to joint
            T1 = T11*T12 %from joint to end-effector
            %for second leg
            T21 = T_2_w*T21 %from base to joint
            T2 = T21*T22 %from joint to end-effector
            %for third leg
            T31 = T_3_w*T31 %from base to joint
            T3 = T31*T32 %from joint to end-effector
            %End of inverse kinematics

            %Transformation. As far as we have only flexible links and the joints are
            %passive, so, we need to include only 6 reference frames from links, 6 from
            % passive joints, 1 from end-effector. T1, T2, T3 - total tranformation
            % matrixes for each leg

            %Jacobians.
            %For the first leg
            %first joint
            J10 = [0 0 ef(3) 0 0 0]
            %second joint
            J11 = [T11(1,4) T11(2,4) T11(3,4) 0 0 1]
            %end-effector
            J12 = [T1(1,4) T1(2,4) T1(3,4) 0 0 1]
            %For the second leg
            %first joint
            J20 = [0 ef(2) 0 0 0 0]
            %second joint
            J21 = [T21(1,4) T21(2,4) T21(3,4) 0 0 1]
            %end-effector
            J22 = [T2(1,4) T2(2,4) T2(3,4) 0 0 1]
            %For the third leg
            %first joint
            J30 = [ef(1) 0 0 0 0 0]
            %second joint
            J31 = [T31(1,4) T31(2,4) T31(3,4) 0 0 1]
            %end-effector
            J32 = [T3(1,4) T3(2,4) T3(3,4) 0 0 1]

            Jq = [J10;J11;J20;J21;J30;J31]'
            Jtheta = [J11; J12; J21; J22; J31; J32]'
            zero = zeros(6,6)

            phi_2_1 = atan2(ef(2), ef(1)) %angle of the first roational joint
            d = 300e-3;
            l = 300e-3;
            L = 100e-3;
            e = -2*l*(ef(1)+d/2);

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

            k = [E*S/L 0                 0                 0           0                 0;
                0           12*E*Iz/L^3  0                 0           0                 6*E*Iy/L^2;
                0           0                  12*E*Iy/L^3 0           -6*E*Iy/L^2 0;
                0           0                  0                 G*J/L 0                 0;
                0           0                  -6*E*Iy/L^2 0           4*E*Iy/L      0;
                0           6*E*Iy/L^2   0                 0           0                 4*E*Iz/L];

            A = [zero    Jtheta   Jq
                 Jtheta' -1\k     zero
                 Jq'     zero     zero]
            A = inv(A+ eye(18)*1e-3)  %seems like we will do without it
            Kc = A(1:6,1:6)
            Kc_theta = A(6:12,1:6)
            Kc_q = A(12:18,1:6)
            %introducing F
            F = [100 0 0 0 0 0]'
            delta_t = inv(Kc)*F
            dt=Kc*F;
            dr=sqrt(dt(1)^2+dt(2)^2+dt(3)^2);
            z1(i,j,kk) = dr;
            
            F = [0 100 0 0 0 0]'
            delta_t = inv(Kc)*F
            dt=Kc*F;
            dr=sqrt(dt(1)^2+dt(2)^2+dt(3)^2);
            z2(i,j,kk) = dr;
            
            F = [0 0 100 0 0 0]'
            delta_t = inv(Kc)*F
            dt=Kc*F;
            dr=sqrt(dt(1)^2+dt(2)^2+dt(3)^2);
            z3(i,j,kk) = dr;
            
            F = [0 0 0 0 0 0]'
            delta_t = inv(Kc)*F
            dt=Kc*F;
            dr=sqrt(dt(1)^2+dt(2)^2+dt(3)^2);
            z0(i,j,kk) = dr;
            
            F = [100 100 100 0 0 0]'
            delta_t = inv(Kc)*F
            dt=Kc*F;
            dr=sqrt(dt(1)^2+dt(2)^2+dt(3)^2);
            z4(i,j,kk) = dr;
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
surf(x,y,z1(:,:,1))
title("deflection for 100N along x for XY")
zx = reshape(z1(1,:,:), 10, 10)
subplot(1,3,2);
surf(y,z,zx)
title("deflection for 100N along x for YZ")
zy = reshape(z1(:,1,:), 10, 10)
subplot(1,3,3);
surf(x,z,zy)
title("deflection for 100N along x for XZ")

figure(2)
subplot(1,3,1);
surf(x,y,z2(:,:,1))
title("deflection for 100N along y for XY")
zx = reshape(z2(1,:,:), 10, 10)
subplot(1,3,2);
surf(y,z,zx)
title("deflection for 100N along y for YZ")
zy = reshape(z2(:,1,:), 10, 10)
subplot(1,3,3);
surf(x,z,zy)
title("deflection for 100N along y for XZ")

figure(3)
subplot(1,3,1);
surf(x,y,z3(:,:,1))
title("deflection for 100N along z for XY")
zx = reshape(z3(1,:,:), 10, 10)
subplot(1,3,2);
surf(y,z,zx)
title("deflection for 100N along z for YZ")
zy = reshape(z3(:,1,:), 10, 10)
subplot(1,3,3);
surf(x,z,zy)
title("deflection for 100N along z for XZ")

figure(4)
subplot(1,3,1);
surf(x,y,z0(:,:,1))
title("deflection without force for XY")
zx = reshape(z0(1,:,:), 10, 10)
subplot(1,3,2);
surf(y,z,zx)
title("deflection without force for YZ")
zy = reshape(z0(:,1,:), 10, 10)
subplot(1,3,3);
surf(x,z,zy)
title("deflection without force for XZ")

figure(5)
subplot(1,3,1);
surf(x,y,z4(:,:,1))
title("deflection with all the forces for XY")
zx = reshape(z4(1,:,:), 10, 10)
subplot(1,3,2);
surf(y,z,zx)
title("deflection with all the forces for YZ")
zy = reshape(z4(:,1,:), 10, 10)
subplot(1,3,3);
surf(x,z,zy)
title("deflection with all the forces for XZ")

theta = Kc_theta*delta_t
% subplot(2,2,1)



