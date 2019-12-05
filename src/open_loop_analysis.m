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

%% Useful Constants

dt = 0.01;
x0_long = [2.058*pi/180 ; 600.0; 2.058*pi/180; 0.0];
x0_lat = [0.0; 0.0; 0.0; 0.0];   

long_vars = ["Pitch Angle - \theta [ \circ ]", "Velocity - V_t [m/s]" ,"Angle of Attack - \alpha [ \circ ]", "Pitch Rate - q [ \circ/s]"]; 
lat_vars = ["Roll Angle - \phi [ \circ ]", "Side-Slip Angle - \beta [ \circ ]", "Roll Rate - p [\circ/s]", "Yaw Rate - r [\circ/s]"];

opt = stepDataOptions;
opt.StepAmplitude = -1;


%% Periodic Motions
%% Short Period
t_short = 0:dt:10;
u_short = 25*pi/180*[ zeros(1,numel(t_short)) ; 
            -1 zeros(1,numel(t_short)-1)];



%[y_short, x_short] = lsim(A_long, B_long, C_long, D_long, u_short, t_short, x0_long);
y_short = transpose(x0_long) + impulse(sys_long(:,2),  t_short, opt)*-1;

%% Phugoid
t_phugoid = 0:dt:1500;
u_phugoid = 25*[ zeros(1,numel(t_phugoid)) ; 
              1 zeros(1,numel(t_phugoid)-1)];
          
%[y_phugoid, x_phugoid] = lsim(A_long, B_long, C_long, D_long, u_phugoid, t_phugoid, x0_long);
y_phugoid = transpose(x0_long) + impulse(sys_long(:,2),  t_phugoid, opt)*-1;

%% Dutch Roll

t_dutch = 0:dt:25;
u_dutch = 25*[ zeros(1,numel(t_dutch)) ; 
            zeros(1,numel(t_dutch)) ; 
            1 zeros(1,numel(t_dutch)-1)];

%[y_dutch, x_dutch] = lsim(A_lat, B_lat, C_lat, D_lat, u_dutch, t_dutch, x0_lat);
y_dutch = impulse(sys_lat(:,3), t_dutch, opt)*-1;


%% Aperiodic Motions
%% Aperiodic Roll

t_aroll = 0:dt:10;
u_aroll = 25*[ zeros(1,numel(t_aroll)) ; 
            1 zeros(1,numel(t_aroll)-1) ; 
            zeros(1,numel(t_aroll))];

%[y_aroll, x_aroll] = lsim(A_lat, B_lat, C_lat, D_lat, u_aroll, t_aroll, x0_lat);
y_aroll = impulse(sys_lat(:,2), t_aroll, opt)*-1;
%% Spiral

t_spiral = 0:dt:1000;
u_spiral = 25*[ zeros(1,numel(t_spiral)) ; 
            1 zeros(1,numel(t_spiral)-1) ; 
            zeros(1,numel(t_spiral))];
%opt.StepAmplitude = 1;
%[y_spiral, x_spiral] = lsim(A_lat, B_lat, C_lat, D_lat, u_spiral, t_spiral, x0_lat);
y_spiral = impulse(sys_lat(:, [2 3]), t_spiral, opt)*-1;

%% Plots
%  Short Period - Phugoid - Dutch Roll - Aperiodic Roll - Spiral
to_plot = [ 1 1 1 1 1 ];

if to_plot(1)
    figure
    for i = 1:4
        subplot(2, 2, i);
        plot(t_short, y_short(:, i), 'b-');
        grid on
        title(long_vars(i));
        xlabel("Time [s]")
        ylabel(long_vars(i));
    end
    sgtitle("Short Period");
end

if to_plot(2)
    figure
    for i = 1:4
        subplot(2, 2, i);
        plot(t_phugoid, y_phugoid(:, i), 'b-');
        grid on
        title(long_vars(i));
        xlabel("Time [s]")
        ylabel(long_vars(i));
    end
    sgtitle("Phugoid");
end

if to_plot(3)
    figure
    for i = 1:4
        subplot(2, 2, i);
        plot(t_dutch, y_dutch(:, i), 'b-');
        grid on
        title(lat_vars(i));
        xlabel("Time [s]")
        ylabel(lat_vars(i));
    end
    sgtitle("Dutch Roll");
    
    figure 
    plot(y_dutch(:,4), y_dutch(:,3));
    grid on
    title("Dutch Roll - Roll Rates");
    xlabel("Roll Rate - \circ/s")
    ylabel("Yaw Rate - \circ/s");
    
end


if to_plot(4)
    figure
    for i = 1:4
        subplot(2, 2, i);
        plot(t_aroll, y_aroll(:, i), 'b-');
        grid on
        title(lat_vars(i));
        xlabel("Time [s]")
        ylabel(lat_vars(i));
    end
    sgtitle("Aperiodic Roll");
end

if to_plot(5)
    figure
    for i = 1:4
        subplot(2, 2, i);
        plot(t_spiral, y_spiral(:, i), 'b-');
        grid on
        title(lat_vars(i));
        xlabel("Time [s]")
        ylabel(lat_vars(i));
    end
    sgtitle("Spiral");
end





