
%%%Calculations of chapter 5
%(run FinfF16Dynamics first)
%% Matrices for accelerometer model, with xa=0 (or with the corresponding xa)
A_accelerometer = A_longitude_lo; %same as longitudinal model
B_accelerometer = B_longitude_lo; %same as longitudinal model
C_accelerometer = mat_lo([21 23 25 26 29 37], [3 5 7 8 11 13 14]); %row 37 added(row 19 of C)
D_accelerometer = mat_lo([21 23 25 26 29 37], [19 20]); %row 37 added(row 19 of D)

  %% 5.3 The linearized output equation y = C*x + D*u
    % Taking the row 19 of C and D => C(19,:),D(19,:)
    % but D as no non-zero components in that row, so only C(19,:)  
  

%% 5.5 Convert to transfer function(elevator to accelerometer) and save for each xa case

s = tf('s');
ACC_LTI = ss(A_accelerometer,B_accelerometer,C_accelerometer,D_accelerometer);
sys = tf(ACC_LTI);

  %%%%%Here we saved all the TF corresponding to each position of the
  %%%% accelerometer
    % save('TFxa0','sys')
  %%%%%%%%  
  % load('TFx0')
  % TF_of_x0 = tf(sys.Numerator(6,2),sys.Denominator(6,2))

   %% 5.6 Step response of the system with xa=0
   
   dt = 0.005;
   t = 0:dt:0.5;
   % Step response, first 0.5 seconds
   y = step(-ACC_LTI(6,2),t);
   figure
   plot(t,y);
   title 'Normal acceleration response to a negative step elevator command'
   xlabel('Time [s]');
   ylabel('a_{n} [g units]');
   %Step response,first 15 seconds
   t = 0:dt:15;
   y = step(-ACC_LTI(6,2),t);
   figure
   plot(t,y);
   title 'Normal acceleration response to a negative step elevator command'
   xlabel('Time [s]');
   ylabel('a_{n} [g units]');

   %% 5.7 Indentify which transfer function component forces 
   %%the step response to be initially in the opposite direction
   
   % => write the TF in the zero pole gain form
   zpk(sys(6,2))
   
 %% 5.9 Transfer functions for different positions of xa
   
   % Xa=0
   load('TFxa0')
   TF_xa0 = tf(sys.Numerator(6,2),sys.Denominator(6,2))
   Zeros_xa0 = zero(TF_xa0);
   
   % Xa=5ft(1,524m)
   load('TFxa5')
   TF_xa5 = tf(sys.Numerator(6,2),sys.Denominator(6,2))
   Zeros_xa5 = zero(TF_xa5);
   zpk(TF_xa5)
   
   % Xa=5,9ft(1,7988m)
   load('TFxa59')
   TF_xa59 = tf(sys.Numerator(6,2),sys.Denominator(6,2))
   Zeros_xa59 = zero(TF_xa59);
   zpk(TF_xa59)
   
   % Xa=6ft(1,8288m)
   load('TFxa6')
   TF_xa6 = tf(sys.Numerator(6,2),sys.Denominator(6,2))
   Zeros_xa6 = zero(TF_xa6);
   zpk(TF_xa6)
   
   % Xa=7ft(2,1336m)
   load('TFxa7')
   TF_xa7 = tf(sys.Numerator(6,2),sys.Denominator(6,2))
   Zeros_xa7 = zero(TF_xa7);
   zpk(TF_xa7)
   
   % Xa=15ft(4,572m)
   load('TFxa15')
   TF_xa15 = tf(sys.Numerator(6,2),sys.Denominator(6,2))
   Zeros_xa15 = zero(TF_xa15);
   zpk(TF_xa15)
   
% Step response comparison( to obtain the zoom just change the final value
% of t from 0.5 to 0.05)

dt = 0.005;
t = 0:dt:0.5;
y1=step(-TF_xa0,t)
y2=step(-TF_xa5,t)
y3=step(-TF_xa59,t)
y4=step(-TF_xa6,t)
y5=step(-TF_xa7,t)
y6=step(-TF_xa15,t)
figure()
plot(t,y1,t,y2,t,y3,t,y4,t,y5,t,y6)
title 'Normal acceleration response to a negative step elevator command'
legend('Xa=0ft','Xa=5ft','Xa=5.9ft','Xa=6ft','Xa=7ft','Xa=15ft')
xlabel 'Time [s]'
ylabel 'a_{n} [g units]'

   
   

