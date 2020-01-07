%------------- Linearised System Matrices ---------------%
A_OBXT2 = [ 0, 1,         0, 0,        0, 0;
 0, 0,    -49/50, 0,   -49/50, 0;
 0, 0,         0, 1,        0, 0;
 0, 0, -539/1000, 0, -49/1000, 0;
 0, 0,         0, 0,        0, 1;
 0, 0,   -49/500, 0, -539/500, 0];
B_OBXT2 = [0;
  1/1000;
       0;
 1/20000;
       0;
 1/10000];

%------------- Observability for theta1(t), theta2(t) ---------------%
CXT2 = [1 0 0 0 0 0;
        0 0 0 0 1 0;
    ];
%For the system to be controllable, the rank must be equal to matrix A rank
if length(A_OBXT2) == rank(obsv(A_OBXT2, CXT2))
    disp('The system is Observable for theta1(t) and theta2(t)');
else
    disp("System is not Observable for theta1(t) and theta2(t)");
end

%-------------- Observer -------------------%
R_OBXT2 = 0.00001;
Q_OBXT2 = [1 0 0 0 0 0;
         0 1 0 0 0 0;
         0 0 500 0 0 0; 
         0 0 0 700 0 0; 
         0 0 0 0 100 0; 
         0 0 0 0 0 2500];
K_OBXT2 = lqr(A_OBXT2, B_OBXT2, Q_OBXT2, R_OBXT2);
xInitial = [3; 0; 0.5; 0; 0.5; 0; 0; 0; 0; 0; 0; 0];
POLES = [-0.1 -0.2 -0.3 -0.4 -0.5 -0.6];
L = place(transpose(A_OBXT2), transpose(CXT2), POLES);
L = transpose(L);
EIGEN_VALUES = eig(A_OBXT2 - L*CXT2);
Ac = [(A_OBXT2-B_OBXT2*K_OBXT2) (B_OBXT2*K_OBXT2);(zeros(size(A_OBXT2))) (A_OBXT2 - L*CXT2)];
Bc = [B_OBXT2; zeros(size(B_OBXT2))];
Cc = [CXT2 zeros(size(CXT2))];
Dc = 0;
OBS_SYS = ss(Ac, Bc, Cc, Dc)
initial(OBS_SYS, xInitial);
xlabel('Time')
ylabel(' Theta2                             X')
grid

hold on
figure(2);
step(OBS_SYS)
xlabel('Time')
ylabel(' Theta2                             X')
hold off