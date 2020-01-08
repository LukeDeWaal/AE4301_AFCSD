
%%% Calculations of chapter 8
%(run FinfF16Dynamics first)
% %% Matrices for glideslope controller,states:h,V_t,alpha,theta,q
% A_glideslope =  mat_lo([3 7 8 5 11], [3 7 8 5 11]);
% B_glideslope =  mat_lo([3 7 8 5 11],[13 14]);
% C_glideslope = mat_lo([21 25 26 23 29],[3 7 8 5 11]);
% D_glideslope = zeros(5,2);
% 
% %Linear time invariant system 
% Glideslope_LTI = ss(A_glideslope,B_glideslope,C_glideslope,D_glideslope);
   %Here we saved the LTI in order to use the matrices in the Simulink
   %model
        %save('GS_LTI','Glideslope_LTI');
        %load('GS_LTI') 

% Initial conditions:thrust = 2826.8165 lb,elevator = -4.1891 deg,ail = -1.9926e-15 deg
% rud = 1.2406e-14 deg,alpha  = 10.4511 deg

  %Here we saved all the outputs coming from the Simulink model
       %save('output_GS','out')

 load('output_GS')
%    dt = 0.1;
%    t = 0.1:dt:151.3;

%% Plots:
  %Height to runway vs time
   figure
   plot(out.Altitude(:,1),out.Altitude(:,2));
   xline(10);
   xline(145.3);
   txt = 'Glideslope interception';
   text(10,2000,txt);
   txt = 'Flare';
   text(145,62,txt);
   title 'Height to runway during glideslope and flare'
   xlabel('Time [s]');
   ylabel('Height to runway [ft]');
   
  %Glideslope error angle vs time
   figure
   plot(out.GS_error(1:1472,1),out.GS_error(1:1472,2));
   xline(10);
   xline(145.3);
   txt = 'Glideslope interception';
   text(10,0,txt);
   txt = 'Flare';
   text(145,0,txt);
   title 'Glideslope error angle \Gamma'
   xlabel('Time [s]');
   ylabel('Glideslope error angle \Gamma [deg]');
   %When R(slant range) is small, the system might become unstable(see lecture slides)
   %but the oscillations are small so we did not introduce a compensator

  %Theta,alpha and flight path angle(gamma) during glideslope
  figure
  plot(out.alpha(:,1),out.alpha(:,2),out.theta(:,1),out.theta(:,2),out.theta(:,1),out.theta(:,2)-out.alpha(:,2));
  xline(10);
  xline(145.3);
  txt = 'Glideslope interception';
  text(10,0,txt);
  txt = 'Flare';
  text(145,0,txt);
  title 'Angle of attack \alpha, pitch angle \theta and flight path angle \gamma '
  legend('\alpha','\theta','\gamma')
  xlabel('Time [s]');
  ylabel('Angles [deg]');
  
  % Total speed V_t during the glideslope and flare
  figure
  plot(out.Speed(:,1),out.Speed(:,2));
  xline(10);
  xline(145.3);
  txt = 'Glideslope interception';
  text(10,300,txt);
  txt = 'Flare';
  text(145,300,txt);
  title 'Total speed V_{t} during glideslope and flare '
  xlabel('Time [s]');
  ylabel('Total speed V_{t} [ft/s]');
  
  %Vertical speed during glideslope and flare
  figure
  plot(out.vertical_speed(:,1),out.vertical_speed(:,2))
  xline(10);
  xline(145.3);
  txt = 'Glideslope interception';
  text(10,0,txt);
  txt = 'Flare';
  text(145,0,txt);
  title 'Vertical speed during glideslope and flare '
  xlabel('Time [s]');
  ylabel('Vertical speed [ft/s]');
  
  %Vertical speed during flare
  figure
  plot(out.vertical_speed(1472:1538,1),out.vertical_speed(1472:1538,2))
  xline(145.3);
  xline(151.243);
  yline(-2.13);
  txt = 'Flare start';
  text(145.3,0,txt);
  txt = 'Touchdown';
  text(151.243,0,txt);
  txt = 'design touchdown speed = 2.13 ft/s';
  text(147,-2,txt);
  title 'Vertical speed during flare '
  xlabel('Time [s]');
  ylabel('Vertical speed [ft/s]');
  
  % Height to runway during flare
  figure
  plot(out.Altitude(1472:1538,1),out.Altitude(1472:1538,2),out.Altitude(1472:1538,1),62.89*exp(-(out.Altitude(1472:1538,1)-145.3)/4));
  xline(145.3);
  xline(151.243);
  txt = 'Flare start';
  text(145,10,txt);
  txt = 'Touchdown';
  text(151.24,10,txt);
  txt = 'design flare path h=h_{flare}e^{-t/\tau}';
  text(147,10,txt);
  title 'Height to runway during flare '
  xlabel('Time [s]');
  ylabel('Height to runway [ft]');
  
  % Height to runway during flare
  figure
  plot((out.Altitude(1472:1538,1)-145.3)*300,out.Altitude(1472:1538,2))
  xline(1200);
  txt = 'Runway threshold';
  text(1200,10,txt);
  title 'Flare Trajectory '
  xlabel('Ground position(origin at flare start) [ft]');
  ylabel('Height to runway [ft]');




