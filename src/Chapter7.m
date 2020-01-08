clear()
% load ABCD_ac Matrices from Chapter 6
load('ABCD_ac_Matrices.mat')

% 2 state short period reduced model matrices
A_sp_2 = A_ac_long(3:4,3:4);
B_sp_2 = B_ac_long(3:4,2);
C_sp_2 = C_ac_long(3:4,3:4);
D_sp_2 = D_ac_long(3:4,2);

% 4 state short period reduced model matrices
A_sp_4 = A_ac_long;
B_sp_4 = B_ac_long(:,2);
C_sp_4 = C_ac_long;
D_sp_4 = D_ac_long(:,2);

% create 2 state space systems for 2 and 4 state short period reduced model
sp_2 = ss(A_sp_2,B_sp_2,C_sp_2,D_sp_2);
sp_4 = ss(A_sp_4,B_sp_4,C_sp_4,D_sp_4);

%Open Loop Transfer Functions from Delta e to q
tf_2 = tf(sp_2(2));
tf_4 = tf(sp_4(4));

% change sign of step function to be negative to get positive response
opt = stepDataOptions('StepAmplitude',-1);

figure

step(sp_2(2),'b--',sp_4(4),'r',10,opt)
grid on;
title('Pitch Rate Response to Negative Step Input');
ylabel('q [°/s]')
xlabel('t')
legend('2 State Model','4 State Model')

% flight conditions and controller design criterion
h = 10000*0.3048;
v = 600*0.3048;
g = 9.80665;

%gibson and dropback constraints
omega_np_sp = 0.03*v;
T_theta_2 = 1/(0.75*omega_np_sp);
zeta_sp = 0.5;

% desired pole location to meet controller design
p = [-zeta_sp*omega_np_sp + omega_np_sp*sqrt(zeta_sp^2-1),
    -zeta_sp*omega_np_sp - omega_np_sp*sqrt(zeta_sp^2-1)];

% requiered controller gains to move poles to desired location
K = place(sp_2.A,sp_2.B,p);

% state space system with feedback
sp_2_fb = ss(sp_2.A-sp_2.B*K,B_sp_2,C_sp_2,D_sp_2);
pole(sp_2_fb)

% close loop transfer function from delta e to q
tf_2_fb = tf(sp_2_fb(2));

% calculate gust deflection
v_gust = 4.572;

% elevator deflection from gust
alpha_e_gust = atan(v_gust/v);
delta_e_gust = alpha_e_gust*K(1);

T_theta_2_old = tf_2_fb.Numerator{1,1}(2)/tf_2_fb.Numerator{1,1}(3);

% lag lead filter
s = tf('s');
laglead = (1+T_theta_2*s)/(1+T_theta_2_old*s);
tf_2_filter = minreal(laglead*tf_2_fb);

% CAP criterion graphs
CAP_1 = [0.25,0.28,1.3-0.25,3.6-0.28];
CAP_2 = [0.2,0.085,2-0.2,3.6-0.085];
CAP_3 = [0.25,0.16,1.3-0.25,3.6-0.16];

CAP = (omega_np_sp^2)/((v/g)*(1/T_theta_2));

figure
for i = 1:3
    subplot(2,2,i)
    grid on;
    rect = rectangle('Position',eval('CAP_'+string(i)),'FaceColor',[0 1 0 0.4]);hold on;
    scatter(zeta_sp,CAP,'filled','r');
    set(gca, 'YScale','log')
    set(gca, 'XScale','log')
    xlim([0.1 10])
    ylim([0.01,10])
    set(gca,'YTick',[0.01,0.1,1,10,],...
            'YTickLabel',{'0.01' '0.1' '1' '10'})
    set(gca,'XTick',[0.1,1,10,],...
            'XTickLabel',{'0.1' '1' '10'});
    title('Flight Phase Category ' + string(char('A' -1 + i)));
    ylabel('CAP [1/g \cdot s^{2}]')
    xlabel('\zeta_{sp} [-]')
end

% pitch rate graph
figure
t = 0:0.01:20;
u = (heaviside(t)-heaviside(t-10));
[fb_y,fb_x] = lsim(-1*tf_2_filter,u,t);
plot(fb_x,fb_y); hold on;
plot([0,9.9999,10,20],[1,1,0,0]);
grid on

% pitch angle graph
figure
t = 0:0.01:20;
u = (heaviside(t)-heaviside(t-10));
[aq1,aq2] = lsim(-1*(1/s)*tf_2_filter,u,t);
plot(aq2,aq1); hold on;
plot([0,10,20],[0,3.2090,3.2090])
grid on

% dropback criterion
DB_qss = T_theta_2 - (2*zeta_sp)/(omega_np_sp);
qm_qs = max(fb_y)/fb_y(999);

% dropback graph
figure
trg_x = [0 0.3 0.06 0];
trg_y = [1 1 3 3];
patch(trg_x,trg_y,[0 1 0],'FaceAlpha',.4);hold on;
scatter(DB_qss,qm_qs,'filled','r');
grid on
xlim([0,0.6])
ylim([1,4])
xlabel('DB/q_{s} [s]')
ylabel('q_{m}/q_{s} [-]')
