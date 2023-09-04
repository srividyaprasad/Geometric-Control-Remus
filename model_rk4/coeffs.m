% File taken from Trim Calculation Methods for a Dynamical Model of the REMUS 100 Autonomous Underwater Vehicle by Raewyn Hall and Stuart Anstee
% REMUS Hydrodynamic Coefficients
% Daniel Sgarioto, DTA
% Nov 2006

global V scale
% Vehicle Parameters
%
U0=V;
m = 30.48;
g = 9.81;
%
W = m*g;
B = W + (0.75*4.44822162);
L = 1.3327;
%
zg = 0.0196;
%
Ixx = 0.177;
Iyy = 3.45;
Izz = 3.45;
%
cdu = 0.2;
rho = 1030;
Af = 0.0285;
d = 0.191;
xcp = 0.321;
Cydb = 1.2;
%
mq = 0.3;
%
cL_alpha = 3.12;
Sfin = 0.00665;
xfin = -0.6827;
%
gamma = 1;
a_prop = 0.25;
w_prop = 0.2;
tau = 0.1;
Jm = 1;
%
l_prop = 0.8*0.0254;
d_prop = 5.5*0.0254;
A_prop = (pi/4)*(d_prop^2);
m_f = gamma*rho*A_prop*l_prop;
%
Kn = 0.5;
%
% Most are Prestero's estimates, but revised values of some linear
% coefficients are due to Fodrea. Thruster coeffs based on results 
% reported by Allen et al.
%
Xwq= -35.5;
Xqq= -1.93;
Xvr= 35.5;
Xrr= -1.93;
Yvv= -1310.0;
Yrr= 0.632;
Yuv= -28.6;
Yur= 5.22;
Ywp= 35.5;
Ypq= 1.93;
Yuudr= 9.64;
Zww= -1310.0;
Zqq= -0.632;
Zuw= -28.6;
Zuq= -5.22;
Zvp= -35.5;
Zrp= 1.93;
Zuuds= -9.64;
Kpp= -0.130;
Mww= 3.18;
Mqq= -188;
Muw= 24.0;
Muq= -2.0;
Mvp= -1.93;
Mrp= 4.86;
Muuds= -6.15;
Nvv= -3.18;
Nrr= -94.0;
Nuv= -24.0;
Nur= -2.0;
Nwp= -1.93;
Npq= -4.86;
Nuudr= -6.15*scale;
Kpdot= -0.0704;
%
Xuu = -0.5*rho*cdu*Af;
Xu = -rho*cdu*Af*U0;
%
% Added Mass Coeffs
Xudot= -0.93;
%
Yvdot= -35.5;
Yrdot= 1.93;
%
Zwdot= -35.5;
Zqdot= -1.93;
%
Mwdot= -1.93;
Mqdot= -4.88;
%
Nvdot= 1.93;
Nrdot= -4.88;
%
% Added Mass Terms
%
Zwc = -15.7;
Zqc = 0.12;
%
Mwc = -0.403;
Mqc = -2.16;
%
% Added Mass Coupling Terms
Xqa = Zqdot*mq;
Zqa = -Xudot*U0;
Mwa = -(Zwdot - Xudot)*U0;
Mqa = -Zqdot*U0;
%
% Body Lift Contribution
%
Zwl = -0.5*rho*(d^2)*Cydb*U0;
%
Mwl = -0.5*rho*(d^2)*Cydb*xcp*U0;
%
% Fin Contribution
%
Zwf = -0.5*rho*cL_alpha*Sfin*U0;
Zqf = 0.5*rho*cL_alpha*Sfin*xfin*U0;
%
Mwf = 0.5*rho*cL_alpha*Sfin*xfin*U0;
Mqf = -0.5*rho*cL_alpha*Sfin*(xfin^2)*U0;
%
% Dive Plane Coeffs
Zw = Zwc + Zwl + Zwf;
Zq = 2.2;
%
Mw = -9.3;
Mq = Mqc +Mqa +Mqf;
%
% Steering Coeffs
Yv = Zw;
Yr = 2.2;
%
Nv = -4.47;
Nr = Mq;
%
% Control Surface Coeffs
Zds = -rho*cL_alpha*Sfin*(U0^2);
Mds = rho*cL_alpha*Sfin*xfin*(U0^2);
%
Ydr = -Zds/3.5;
Ndr = Mds/3.5;
%
% Thruster Coeffs
Tnn = 6.279e-004;
Tnu = 0; 
%
Qnn = -1.121e-005;
Qnu = 0;
%
Tnu0 = (1/(a_prop + 1))*Tnu;
Qnu0 = (1/(a_prop + 1))*Qnu;
%
df0 = (-Xu)/((1 - tau)*(1 + a_prop)*(1 - w_prop));
df = (-Xuu)/((1 - tau)*(1 + a_prop)*a_prop*((1 - w_prop)^2));
%