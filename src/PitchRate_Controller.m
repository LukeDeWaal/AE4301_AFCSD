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

tf_2 = tf(sp_2(2))
tf_4 = tf(sp_4(4))

poles_2=roots(cell2mat(tf_2.Den));
zeros_2=roots(cell2mat(tf_2.Num));

poles_4=roots(cell2mat(tf_4.Den));
zeros_4=roots(cell2mat(tf_4.Num));

%clearvars -except sp_2 sp_4;

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

omega_np_sp = 0.03*v;
T_theta_2 = 1/(0.75*omega_np_sp);
zeta_sp = 0.5;

% desired pole location to meet controller design
p = [-zeta_sp*omega_np_sp + omega_np_sp*sqrt(zeta_sp^2-1),
    -zeta_sp*omega_np_sp - omega_np_sp*sqrt(zeta_sp^2-1)];

% requiered controller gains to move poles to desired location
K = place(sp_2.A,sp_2.B,p);

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


