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


%B_lat(2,2) = 0.00051;
%B_lat(3,2) = 0.07550;

sys_long = ss(A_long, B_long, C_long, D_long);
sys_lat = ss(A_lat, B_lat, C_lat, D_lat);

%% Motion Characteristics

p_long = pole(sys_long);
p_lat = pole(sys_lat);

[wn_long,zeta_long] = damp(sys_long);
[wn_lat,zeta_lat] = damp(sys_lat);

wn_sp = wn_long(3); % Short Period
zeta_sp = zeta_long(3);
wd_sp = damped_freq(wn_sp, zeta_sp);
Th_sp = halfamp_period(wn_sp, zeta_sp);
P_sp = (2*pi)/wn_sp;
wn_ph = wn_long(1); % Phugoid
zeta_ph = zeta_long(1);
wd_ph = damped_freq(wn_ph, zeta_ph);
Th_ph = halfamp_period(wn_ph, zeta_ph);
P_ph = (2*pi)/wn_ph;

wn_dr = wn_lat(3); % Dutch Roll
zeta_dr = zeta_lat(3);
wd_dr = damped_freq(wn_dr, zeta_dr);
Th_dr = halfamp_period(wn_dr, zeta_dr);
P_dr = (2*pi)/wn_dr;
wn_ar = wn_lat(2); % Aperiodic Roll
zeta_ar = zeta_lat(2);
Th_ar = halfamp_period(wn_ar, zeta_ar);
tau_ar = time_const(wn_ar, zeta_ar);
wn_sm = wn_lat(1); % Spiral Motion
zeta_sm = zeta_lat(1);
Th_sm = halfamp_period(wn_sm, zeta_sm);
tau_sm = time_const(wn_sm, zeta_sm);

clear wn_long wn_lat zeta_long zeta_lat;

%% Useful Constants

dt = 0.001;
x0_long = [2.058*pi/180 ; 600.0; 2.058*pi/180; 0.0];
x0_lat = [0.0; 0.0; 0.0; 0.0];   

long_vars = ["Pitch Angle - \theta [ \circ ]", "Velocity - V_t [m/s]" ,"Angle of Attack - \alpha [ \circ ]", "Pitch Rate - q [ \circ/s]"]; 
lat_vars = ["Roll Angle - \phi [ \circ ]", "Side-Slip Angle - \beta [ \circ ]", "Roll Rate - p [\circ/s]", "Yaw Rate - r [\circ/s]"];

% opt = stepDataOptions;
% opt.StepAmplitude = -1;


%% Periodic Motions
%% Short Period

t_short = 0:dt:10;
y_short = transpose(x0_long) + impulse(sys_long(:,2),  t_short, opt)*-1;

%% Phugoid

t_phugoid = 0:dt:1000;
y_phugoid = transpose(x0_long) + impulse(sys_long(:,2),  t_phugoid, opt)*-1;

%% Dutch Roll

t_dutch = 0:dt:25;
y_dutch = impulse(sys_lat(:,3), t_dutch, opt)*-1;

%% Aperiodic Motions
%% Aperiodic Roll

t_aroll = 0:dt:10;
y_aroll = impulse(sys_lat(:,2), t_aroll, opt)*-1;

%% Spiral

t_spiral = 0:dt:120;
y_spiral = impulse(sys_lat(:, [2 3]), t_spiral, opt)*-1;

%% Plots
%  Short Period - Phugoid - Dutch Roll - Aperiodic Roll - Spiral
to_plot = [ 0 1 0 0 0 ];



if to_plot(1)
    figure
    for i = 1:4
        set(gca,'FontSize',15);
        subplot(2, 2, i);
        plot(t_short, y_short(:, i), 'b-'); hold on;
        plot(Th_sp, intersection_point(Th_sp, t_short, y_short(:,i)), 'ro'); hold on;
        plot(P_sp,  intersection_point(P_sp, t_short, y_short(:,i)), 'ko'); hold off;
        set(gca,'FontSize',15);
        legend('Response', 'Half-Amplitude Period', 'Period');
        grid on
        title(long_vars(i));
        xlabel("Time [s]")
        ylabel(long_vars(i));
        
    end
    sgtitle("Short Period", "FontSize", 20);
end

if to_plot(2)
    figure
    for i = 1:4
        set(gca,'FontSize',15);
        subplot(2, 2, i);
        plot(t_phugoid, y_phugoid(:, i), 'b-'); hold on;
        plot(Th_ph, intersection_point(Th_ph, t_phugoid, y_phugoid(:,i)), 'ro'); hold on;
        plot(P_ph,  intersection_point(P_ph, t_phugoid, y_phugoid(:,i)), 'ko'); hold off;
        set(gca,'FontSize',15);
        legend('Response', 'Half-Amplitude Period', 'Period');
        grid on
        title(long_vars(i));
        xlabel("Time [s]")
        ylabel(long_vars(i));
    end
    sgtitle("Phugoid", "FontSize", 20);
end

if to_plot(3)
    figure
    for i = 1:4
        set(gca,'FontSize',15);
        subplot(2, 2, i);
        plot(t_dutch, y_dutch(:, i), 'b-'); hold on;
        plot(Th_dr, intersection_point(Th_dr, t_dutch, y_dutch(:,i)), 'ro'); hold on;
        plot(P_dr,  intersection_point(P_dr, t_dutch, y_dutch(:,i)), 'ko'); hold off;
        set(gca,'FontSize',15);
        legend('Response', 'Half-Amplitude Period', 'Period');
        grid on
        title(lat_vars(i));
        xlabel("Time [s]")
        ylabel(lat_vars(i));
    end
    sgtitle("Dutch Roll", "FontSize", 20);
    
    figure 
    set(gca,'FontSize',15);
    plot(y_dutch(:,4), y_dutch(:,3));
    grid on
    set(gca,'FontSize',15);
    title("Dutch Roll - Roll Rates");
    xlabel("Roll Rate - \circ/s")
    ylabel("Yaw Rate - \circ/s");
    
end


if to_plot(4)
    figure
    for i = 1:4
        set(gca,'FontSize',15);
        subplot(2, 2, i);
        plot(t_aroll, y_aroll(:, i), 'b-'); hold on;
        plot(Th_ar, intersection_point(Th_ar, t_aroll, y_aroll(:,i)), 'ro'); hold on;
        plot(tau_ar,  intersection_point(tau_ar, t_aroll, y_aroll(:,i)), 'go'); hold off;
        set(gca,'FontSize',15);
        legend('Response', 'Half-Amplitude Period', 'Time Constant');
        grid on
        title(lat_vars(i));
        xlabel("Time [s]")
        ylabel(lat_vars(i));
    end
    sgtitle("Aperiodic Roll", "FontSize", 20);
end

if to_plot(5)
    figure
    for i = 1:4
        set(gca,'FontSize',15);
        subplot(2, 2, i);
        plot(t_spiral, y_spiral(:, i), 'b-'); hold on;
        plot(Th_sm, intersection_point(Th_sm, t_spiral, y_spiral(:,i)), 'ro'); hold on;
        plot(tau_sm,  intersection_point(tau_sm, t_spiral, y_spiral(:,i)), 'go'); hold off;
        
        set(gca,'FontSize',15);
        legend('Response', 'Half-Amplitude Period', 'Time Constant');
        grid on
        title(lat_vars(i));
        xlabel("Time [s]")
        ylabel(lat_vars(i));
    end
    sgtitle("Spiral", "FontSize", 20);
end


function y = damped_freq(wn, zeta)
    y = wn*sqrt(1 - zeta^2);
end

function y = halfamp_period(wn, zeta)
    y = log(2)/(wn*zeta);
end

function y = time_const(wn, zeta)
    y = 1.0/(wn*zeta);
end

function y = intersection_point(t0, ti, yi)
    
    epsilon = 10000000000.0;
    
    for i = 1:length(ti)
        diff = abs(ti(i) - t0);
        if diff < epsilon
            epsilon = diff;
        else
            y = yi(i-1);
            break;
        end
    end
end
