% REMUS.M Vehicle Simulator, returns the time derivative of the state vector
function [ACCELERATIONS,FORCES] = remus(x,ui)
% TERMS
% ---------------------------------------------------------------------
% STATE VECTOR:
% x = [u v w p q r xpos ypos zpos phi theta psi]'
% Body-referenced Coordinates
% u = Surge velocity [m/sec]
% v = Sway velocity [m/sec]
% w = Heave velocity [m/sec]
% p = Roll rate [rad/sec]
% q = Pitch rate [rad/sec]
% r = Yaw rate [rad/sec]
% Earth-fixed coordinates
% xpos = Position in x-direction [m]
% ypos = Position in y-direction [m]
% zpos = Position in z-direction [m]
% phi = Roll angle [rad]
% theta = Pitch angle [rad]
% psi = Yaw angle [rad]
% INPUT VECTOR
% ui = [deltas delta_r]'
% Control Fin Angles
% delta-s = angle of stern planes [rad]
% delta-r = angle of rudder planes [rad]
% Initialize global variables
%---------------------------------------------------------------------
load vdata ; % W and B, CG and CB coords

load inv-massmatrix ; % Minv matrix

load vehicle-coeffs ; % non-zero vehicle coefficients only

% Output flags
showforces = 10;

% Get and check state variables and control inputs

% Get state variables
u =x(1) ;v =x(2) ; w =x(3) ;
p =x(4) ; q = x(5) ; r = x(6) ; 
phi = x(10) ; theta = x(11) ; psi = x(12);

% Get control inputs
delta_s = ui(1) ; delta_r = ui(2);

% Check control inputs (useful later)
if delta_s > delta_max
delta_s = sign(delta_s)*delta_max;
end 
if delta_r > delta_max
delta_r = sign(delta_r)*delta_max;
end

% Initialize elements of coordinate system transform matrix
c1 = cos(phi); c2 = cos(theta); c3 = cos(psi); 
s1 = sin(phi); s2 = sin(theta); s3 = sin(psi); 
t2 = tan(theta);

% Set total forces from equations of motion
X = -(W-B)*sin(theta) + Xuu*u*abs(u) + (Xwq-m)*w*q + (Xqq + m*xg)*q^2 ...
+ (Xvr+m)*v*r + (Xrr + m*xg)*r^2 -m*yg*p*q - m*zg*p*r ...
+ Xprop;

Y = (W-B)*cos(theta)*sin(phi) + Yvv*v*abs(v) + Yrr*r*abs(r) + ...
Yuv*u*v ...
+ (Ywp+m)*w*p + (Yur-m)*u*r - (m*zg)*q*r + (Ypq - m*xg)*p*q ...
+ Yuudr*u^2*delta_r;

Z = (W-B)*cos(theta)*cos(phi) + Zww*w*abs(w) + Zqq*q*abs(q)+ ...
Zuw*u*w ...
+ (Zuq+m)*u*q + (Zvp-m)*v*p + (m*zg)*p^2 + (m*zg)*q^2 ...
+ (Zrp - m*xg)*r*p + Zuuds*u^2*delta_s;

K = -(yg*W-yb*B)*cos(theta)*cos(phi) - (zg*W-zb*B)*cos(theta)*sin(phi) ...
+ Kpp*p*abs(p) - (Izz-Iyy)*q*r - (m*zg)*w*p + (m*zg)*u*r + Kprop;

M = -(zg*W-zb*B)*sin(theta) - (xg*W-xb*B)*cos(theta)*cos(phi) + Mww*w*abs(w) ...
+ Mqq*q*abs(q) + (Mrp - (Ixx-Izz))*r*p + (m*zg)*v*r - (m*zg)*w*q ...
+ (Muq - m*xg)*u*q + Muw*u*w + (Mvp + m*xg)*v*p ...
+ Muuds*u^2*delta_s;

N = -(xg*W-xb*B)*cos(theta)*sin(phi) - (yg*W-yb*B)*sin(theta) ...
+ Nvv*v*abs(v) + Nrr*r*abs(r) + Nuv*u*v ...
+ (Npq - (Iyy-Ixx))*p*q + (Nwp - m*xg)*w*p + (Nur + m*xg)*u*r ...
+ Nuudr*u^2*delta-r ;

FORCES = [X Y Z K M N]';

ACCELERATIONS =...
[Minv(1,1)*X+Minv(1,2)*Y+Minv(1,3)*Z+Minv(1,4)*K+Milv(1,5)*M+Minv(1,6)*N 
Minv(2,1)*X+Minv(2,2)*Y+Minv(2,3)*Z+Minv(2,4)*K+Minv(2,5)*M+Minv(2,6)*N 
Minv(3,1)*X+Minv(3,2)*Y+Minv(3,3)*Z+Minv(3,4)*K+Minv(3,5)*M+Minv(3,6)*N 
Minv(4,1)*X+Minv(4,2)*Y+Minv(4,3)*Z+Minv(4,4)*K+Minv(4,5)*M+Minv(4,6)*N
Minv(5,1)*X+Minv(5,2)*Y+Minv(5,3)*Z+Minv(5,4)*K+Minv(5,5)*M+Minv(5,6)*N
Minv(6,1)*X+Minv(6,2)*Y+Minv(6,3)*Z+Minv(6,4)*K+Minv(6,5)*M+Minv(6,6)*N 
c3*c2*u + (c3*s2*sl-s3*cl)*v + (s3*sl+c3*cl*s2)*w 
s3*c2*u + (cl*c3+sl*s2*s3)*v + (cl*s2*s3-c3*sl)*w 
-s2*u + c2*sl*v + cl*c2*w 
p + sl*t2*q + cl*t2*r 
cl*q - sl*r
sl/c2*q + cl/c2*r];