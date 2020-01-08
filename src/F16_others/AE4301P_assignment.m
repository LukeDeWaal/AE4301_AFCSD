clc 
clear all 
close all 

%load 'Linear_syst_trim_long_lo' % retrieved from FindF16Dynamics.m
%load 'Linear_syst_trim_lat_lo' % retrieved from FindF16Dynamics.m
load('Dynamics_15000_500_plus_an_output_Xa_0')
%% make the system from loaded matrices 
sys_long_lo_fi = ss(A_longitude_lo, B_longitude_lo, C_longitude_lo, D_longitude_lo); 
sys_lat_lo_fi = ss(A_lateral_lo, B_lateral_lo, C_lateral_lo, D_lateral_lo); 

%% Checking systems with figures from the manual (the phugoid is a little off but I assume that is due to the fact that i iterated the lo_fi model quite a few times
figure()
pzmap(sys_long_lo_fi); 
title 'Longitudal lo fi PZ map'
grid on 

figure()
pzmap(sys_lat_lo_fi); 
title 'Lateral lo fi PZ map'
grid on 
%% longitudinal states 

% first row is states: h, theta, v, alpha, q 
% second row is input:  1 = thrust 
%                       2 = elevator 
figure()
step(sys_long_lo_fi(3,1),2); % would be better to use lsim as a larger input can be given
title 'Altitude vs elevator deflection'
xlabel('Time [sec]')
ylabel('Velocity [m/s]')


%% lateral states 

% first row is states: phi, psi, v, beta, p, r
% second row is input: thrust, flaperons, rudder

figure()
step(sys_lat_lo_fi(4,3),100); % would be better to use lsim as a larger input can be given
title 'Beta vs rudder'
xlabel('Time [sec]')
ylabel('Angle [ft/s]')

%% 5.1.3 Influence of an accelerometer position
%y_Xa = C_lo(19,:)*xu_lo
% y_Xa is   2.2298 for when the above is calculated on the simulink in and
% output 



%% 5.1.4
X_a_dependantcy = C_lo(19,:)
% this shows that the X_a output it dependand on alt,vel,alpha,q,ny.   alt and phi do show in the table but have a very small contribution. however, alt is multiplied by a large number ofcourse. 

%% 5.1.5/6 Determine the elevator-to-normal-acceleration transfer function.
load('Dynamics_15000_500_plus_an_output_Xa_0');
tf_lo = tf(SS_lo);
num = tf_lo.Numerator(19,2);
denom = tf_lo.Denominator(19,2);
tfxa_0 = tf(num,denom);

load('Dynamics_15000_500_plus_an_output_Xa_5');
tf_lo = tf(SS_lo);
num = tf_lo.Numerator(19,2);
denom = tf_lo.Denominator(19,2);
tfxa_5 = tf(num,denom);

load('Dynamics_15000_500_plus_an_output_Xa_5_9');
tf_lo = tf(SS_lo);
num = tf_lo.Numerator(19,2);
denom = tf_lo.Denominator(19,2);
tfxa_5_9 = tf(num,denom);

load('Dynamics_15000_500_plus_an_output_Xa_6');
tf_lo = tf(SS_lo);
num = tf_lo.Numerator(19,2);
denom = tf_lo.Denominator(19,2);
tfxa_6 = tf(num,denom);

load('Dynamics_15000_500_plus_an_output_Xa_7');
tf_lo = tf(SS_lo);
num = tf_lo.Numerator(19,2);
denom = tf_lo.Denominator(19,2);
tfxa_7 = tf(num,denom);

load('Dynamics_15000_500_plus_an_output_Xa_15');
tf_lo = tf(SS_lo);
num = tf_lo.Numerator(19,2);
denom = tf_lo.Denominator(19,2);
tfxa_15 = tf(num,denom);

figure()
step(-tfxa_0,2,'.')
hold on 
step(-tfxa_5,2)
step(-tfxa_5_9,2)
step(-tfxa_6,2)
step(-tfxa_7,2)
step(-tfxa_15,2, '-.')
title 'Normal acceleration due to a step input on the elevator'
legend('x_a = 0','x_a = 5','x_a = 5.9','x_a = 6','x_a = 7','x_a = 15')
xlabel 'Time'
ylabel 'a_n [m/s^2]'

%% 5.1.7 Which transfer function component forces the step response to be initially in the opposite direction of the reference signal?
figure()
pzmap(tf_elevator_an)
% the zero located at 9.27 gives a kick in the other direction at first
% moment. I am not sure what physical component that is. 

%% 5.1.8 Give a physical explanation of this non-minimum-phase behaviour
%no idea yet

%% 5.1.9 


zero_xa_0 = zero(tfxa_0);
zero_xa_5 = zero(tfxa_5);
zero_xa_5_9 = zero(tfxa_5_9);
zero_xa_6 = zero(tfxa_6);
zero_xa_7 = zero(tfxa_7);
zero_xa_15 = zero(tfxa_15);

%% 5.1.10 at what point is the ”instantaneous center of rotation” located? 
 
%load('Dynamics_15000_500_plus_an_output_Xa_15');
diff_range = []
for X_a = 5.56190070535 %:0.000001:5.561955
    
    [SS_lo] = Fun_FindF16Dyamics(X_a);
    tf_lo = tf(SS_lo);
    num = tf_lo.Numerator(19,2);
    denom = tf_lo.Denominator(19,2);
    tfxa_range = tf(num,denom);
    figure()
    [x,y] = step(-tfxa_range,2); 
    plot(y,x)
    z = diff(x)
    diff_range = [diff_range,z]
    diff_range(1)
end


%% question 6  reduce the longitudinal and lateral matrices 
% ill first try to use the modred fucntion to reduce matrices 

% we want to get a longitudinal matrix which only contains Vt, alpha, pitch and pitch rate 

% we want to get a lateral matrix which only contains beta, roll angle,
% roll rate and yaw rate

% we first make the space-state systems from the matrices
sys_long_lo_fi = ss(A_longitude_lo, B_longitude_lo, C_longitude_lo, D_longitude_lo); 
sys_lat_lo_fi = ss(A_lateral_lo, B_lateral_lo, C_lateral_lo, D_lateral_lo); 


% first row is states: h, theta, v, alpha, q 
% second row is input:  1 = thrust 
%                       2 = elevator 

figure()
step(sys_long_lo_fi(2,1),2); % would be better to use lsim as a larger input can be given
title 'Altitude vs elevator deflection'
xlabel('Time [sec]')
ylabel('pitch rate(t)')
%%















