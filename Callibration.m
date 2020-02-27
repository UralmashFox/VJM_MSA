clc
clear all
close all
d1 = [0 2] %limits of the (first) translational link
phi1 = [0 1] %limits of the (second) rotational link
l = 2 %length of the link between rotational and translational
%--------------30 experiments--------------------------
x=500.*rand(1,30)
y=500.*rand(1,30)
z=500.*rand(1,30)
ax =500.*rand(1,30)
ay =500.*rand(1,30)
az =500.*rand(1,30)
dt = [0 0 0 0 0 0]
d2 = [0 2] %limits of the (third) translational link
epsilon = 0
W0 = [0 0 0 0 0 0]
for i = 1:30
    W(1) = x(i)
    W(2) = y(i)
    W(3) = z(i)
    W(4) = ax(i)
    W(5) = ay(i)
    W(6) = az(i)
%------------------------------------------------------
% W = [2 2 2 0 0 15]
%inverse
d2 = dt(3)
phi1 = atan2(dt(1), dt(2))
d1 = abs(dt(1)/cos(phi1))
sigma_p = 25e-9
sigma_phi = 0.25e-3
%Jacobians
T1 = [cos(phi1) -sin(phi1)    0 0
      sin(phi1)    cos(phi1)  0 0
      0            0          1 d1
      0            0          0 1 ]
  
 T2 = [ 0 0   1  0
       -1 0   0  0
        0 -1  0  l+d2
        0 0   0  0 ]
    
 Tef = T1*T2
 %------------------------------
%material and shape parameters
K0 = 1e6; 
K1 = 2e6
K2 = 0.5e6 %Young's modulus

J1 = [0 0 1 0 0 0]
cp = cross([1 0 0], [0 0 l+d2])
J2 = [cp(1) cp(2) cp(3) 0 0 1]
J3 = [1 0 0 0 0 0]
  
Jtheta = [J1 J2 J3]

% Ktheta = [K0 0 0 0 0 0 0 0 0
%           0 K0 0 0 0 0 0 0 0
%           0 0 K0 0 0 0 0 0 0
%           0 0 0 K1 0 0 0 0 0
%           0 0 0 0 K1 0 0 0 0
%           0 0 0 0 0 K1 0 0 0
%           0 0 0 0 0 0 K2 0 0
%           0 0 0 0 0 0 0 K2 0
%           0 0 0 0 0 0 0 0 K2]
Ktheta = [K0 0 0 
          0 K1 0 
          0 0 K2 ]
       
%  Kc = inv(Jtheta*(inv(Ktheta))*Jtheta')
%  W = Kc*dt
 A0 = [(J1*J1'*W0)' (J2*J2'*W0)' (J3*J3'*W0)']
 A01 = A0(1:6)
 A02 = A0(7:12)
 A03 = A0(13:18)
 A = [(J1'*J1*W')' (J2'*J2*W')' (J3'*J3*W')']
 A1 = A(1:6)
 A2 = A(7:12)
 A3 = A(13:18)
%  dt = A*Ktheta
 dt1 = A1*K0
 dt2 = A2*K1
 dt3 = A3*K2
 dt1_prime(i, :) = dt1
 dt2_prime(i, :) = dt2
 dt3_prime(i, :) = dt3
 figure(i)
 hold on
 plot3([0,0], [0,0],[0,dt1_prime(3)], 'Color', [0,0,1], 'LineWidth',2)
 plot3([0,dt3_prime(2)*cos(dt3_prime(6))], [0,dt3_prime(2)*sin(dt3_prime(6))],[dt1_prime(3),dt1_prime(3)], 'Color', [0,0,1], 'LineWidth',2)

 dX = inv(A1'*A1+A2'*A2+A3'*A3+eye(6))*(A1'*dt1+A2'*dt1+A2'*dt1)
 Kc = inv(dX*W'+eye(6))
 K1 = sum(diag(inv(W'*A1+eye(6))*inv(Kc+eye(6))))/6
 K2 = inv(W'*A2+eye(6))*inv(Kc+eye(6))
 K3 = inv(W'*A3+eye(6))*inv(Kc+eye(6))
 covdX = sigma_p^2*inv(A1'*A1+A2'*A2+A3'*A3+eye(6))
 ro = sqrt(sigma_p^2*sum(diag(A*inv(A1*A1'+A2*A2'+A3*A3')*A')))

 dt1 = dt1+ro
 dt2 = dt2+ro
 dt3 = dt3+ro
 dt1_final(i, :) = dt1
 dt2_final(i, :) = dt2
 dt3_final(i, :) = dt3
 
figure(i)
 hold on
 plot3([0,0], [0,0],[0,dt1_final(3)], 'Color', [1,0,0], 'LineWidth',2)
 plot3([0,dt3_final(2)*cos(dt3_final(6))], [0,dt3_final(2)*sin(dt3_final(6))],[dt1_final(3),dt1_final(3)], 'Color', [1,0,0], 'LineWidth',2)

 
 dX = inv(A1'*A1+A2'*A2+A3'*A3+eye(6))*(A1'*dt1+A2'*dt1+A2'*dt1)
 Kc = inv(dX*W'+eye(6))
 K1f(i) = sum(diag(inv(W'*A1+eye(6))*inv(Kc+eye(6))))/6
 K2f(i) = sum(diag(inv(W'*A2+eye(6))*inv(Kc+eye(6))))/6
 K3f(i) = sum(diag(inv(W'*A3+eye(6))*inv(Kc+eye(6))))/6
 
end
  kf1 = sum(K1f)/30
  kf2 = sum(K2f)/30
  kf3 = sum(K3f)/30
    
  