%------------- Linearised System Matrices ---------------%
A_OBX = [ 0, 1,         0, 0,        0, 0;
 0, 0,    -49/50, 0,   -49/50, 0;
 0, 0,         0, 1,        0, 0;
 0, 0, -539/1000, 0, -49/1000, 0;
 0, 0,         0, 0,        0, 1;
 0, 0,   -49/500, 0, -539/500, 0];
B_OBX = [0;
  1/1000;
       0;
 1/20000;
       0;
 1/10000];

%------------- Observability for x(t) ---------------%
CX = [1 0 0 0 0 0;
%       0 0 0 0 0 0;
%       0 0 0 0 0 0;
    ];
%For the system to be controllable, the rank must be equal to matrix A rank
if length(A_OBX) == rank(obsv(A_OBX, CX))
    disp('The system is Observable for x(t)');
else
    disp("System is not Observable for x(t)");
end

%-------------- Observer -------------------%
R_OBX = 0.00001;
Q_OBX = [1 0 0 0 0 0;
         0 1 0 0 0 0;
         0 0 500 0 0 0; 
         0 0 0 700 0 0; 
         0 0 0 0 100 0; 
         0 0 0 0 0 2500];
K_OBX = lqr(A_OBX, B_OBX, Q_OBX, R_OBX);
xInitial = [3; 0; 0.5; 0; 0.5; 0; 0; 0; 0; 0; 0; 0];
POLES = [-1.2 -2.4 -3.6 -4.8 -6 -7.2];
L = place(transpose(A_OBX), transpose(CX), POLES);
L = transpose(L);
EIGEN_VALUES = eig(A_OBX - L*CX);
Ac = [(A_OBX-B_OBX*K_OBX) (B_OBX*K_OBX);(zeros(size(A_OBX))) (A_OBX - L*CX)];
Bc = [B_OBX; zeros(size(B_OBX))];
Cc = [CX zeros(size(CX))];
Dc = 0;
OBS_SYS = ss(Ac, Bc, Cc, Dc)
initial(OBS_SYS, xInitial);
xlabel('Time')
ylabel('X')
grid

hold on
figure(2);
step(OBS_SYS)
hold off