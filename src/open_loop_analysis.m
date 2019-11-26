%% Loading The State Space Matrices for wing-level flight
% @ 10000 ft, 600 ft/s
%%

matrices = open('ABCD_ac_MAtrices.mat');

A_long = matrices.A_ac_long; 
B_long = matrices.B_ac_long;
C_long = matrices.C_ac_long; 
D_long = matrices.D_ac_long;

A_lat = matrices.A_ac_lat; 
B_lat = matrices.B_ac_lat;
C_lat = matrices.C_ac_lat; 
D_lat = matrices.D_ac_lat;

sys_long = ss(A_long, B_long, C_long, D_long);
sys_lat = ss(A_lat, B_lat, C_lat, D_lat);

%% Short Period
t_short = 0:0.001:3;
u_short = [ zeros(1,numel(t_short)) ; 1 zeros(1,numel(t_short)-1)];

[y, x] = lsim(A_long, B_long, C_long, D_long, u_short, t_short);
plot(t_short, y)


