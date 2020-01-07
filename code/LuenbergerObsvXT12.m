%------------- Linearised System Matrices ---------------%
A_OBXT12 = [ 0, 1,         0, 0,        0, 0;
 0, 0,    -49/50, 0,   -49/50, 0;
 0, 0,         0, 1,        0, 0;
 0, 0, -539/1000, 0, -49/1000, 0;
 0, 0,         0, 0,        0, 1;
 0, 0,   -49/500, 0, -539/500, 0];
B_OBXT12 = [0;
  1/1000;
       0;
 1/20000;
       0;
 1/10000];

%------------- Observability for theta1(t), theta2(t) ---------------%
CT12 = [1 0 0 0 0 0
        0 0 1 0 0 0;
        0 0 0 0 1 0;
    ];
%For the system to be controllable, the rank must be equal to matrix A rank
if length(ALI) == rank(obsv(ALI, CT12))
    disp('The system is Observable for theta1(t) and theta2(t)');
else
    disp("System is not Observable for theta1(t) and theta2(t)");
end

%-------------- Observer -------------------%
R_OBXT12 = 0.00001;
Q_OBXT12 = [1 0 0 0 0 0;
         0 1 0 0 0 0;
         0 0 500 0 0 0; 
         0 0 0 700 0 0; 
         0 0 0 0 100 0; 
         0 0 0 0 0 2500];
K_OBXT12 = lqr(A_OBXT12, B_OBXT12, Q_OBXT12, R_OBXT12);
xInitial = [3; 0; 0.5; 0; 0.5; 0; 0; 0; 0; 0; 0; 0];
POLES = [-1.2 -2.4 -3.6 -4.8 -6 -7.2];
L = place(transpose(A_OBXT12), transpose(CT12), POLES);
L = transpose(L);
EIGEN_VALUES = eig(A_OBXT12 - L*CT12);
Ac = [(A_OBXT12-B_OBXT12*K_OBXT12) (B_OBXT12*K_OBXT12);(zeros(size(A_OBXT12))) (A_OBXT12 - L*CT12)];
Bc = [B_OBXT12; zeros(size(B_OBXT12))];
Cc = [CT12 zeros(size(CT12))];
Dc = 0;
OBS_SYS = ss(Ac, Bc, Cc, Dc)
initial(OBS_SYS, xInitial);
xlabel('Time')
ylabel(' Theta2                 Theta1                       X')
grid

hold on
figure(2);
step(OBS_SYS)
xlabel('Time')
ylabel(' Theta2                 Theta1                       X')
hold off