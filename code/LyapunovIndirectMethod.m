%----------- Define Matrices-------------%
syms m1;
syms m2;
syms M;
syms l1;
syms l2;
syms g;
syms k;
A = [0 1 0 0 0 0; 
     0 0 -m1*g/M 0 -m2*g/M 0;
     0 0 0 1 0 0;
     0 0 -(g*(m1+M))/(l1*M) 0 -m2*g/(l1*M) 0;
     0 0 0 0 0 1;     
     0 0 -m1*g/(l2*M) 0 -(g*(m2+M))/(l2*M) 0;
];

B = [0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];

C = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0;
    ];
D = zeros(1,1);
% D = 0;



%--------- Check Controllability for given initial conditions-----------%
M = 1000;
m1 = 100;
m2 = 100;
l1 = 20;
l2 = 10;
g =9.8;
AL = subs(A);
BL = subs(B);
%For the system to be controllable, the rank must be equal to matrix A rank
if length(AL) == rank(ctrb(AL, BL))
    disp('The system is Controllable at Eq. Point');
else
    disp("System is not Controllable at Eq. Point");
end

xInitial = [3; 0; 0.5; 0; 0.5; 0];
R = 0.00001;
% QLI = 20000000 * transpose(C) * C; 
QLI = [1 0 0 0 0 0;
       0 1 0 0 0 0;
       0 0 100 0 0 0; 
       0 0 0 500 0 0; 
       0 0 0 0 250 0; 
       0 0 0 0 0 2000];
 
ALI = [ 0, 1,         0, 0,        0, 0;
 0, 0,    -49/50, 0,   -49/50, 0;
 0, 0,         0, 1,        0, 0;
 0, 0, -539/1000, 0, -49/1000, 0;
 0, 0,         0, 0,        0, 1;
 0, 0,   -49/500, 0, -539/500, 0];
BLI = [0;
  1/1000;
       0;
 1/20000;
       0;
 1/10000];

[KLI,SLI,PLI] = lqr(ALI,BLI,QLI,R);
LIN_SYS = ss(ALI - BLI*KLI, BLI, C, D);
[y,t] = step(LIN_SYS);
plot(t,y);
title('Step Input to Linear System')
xlabel('Time')
ylabel('Theta2                  Theta1                X')

%------------------ Lyapunov's Indirect Method ------------------------%
lambda = eig(ALI - BLI * KLI)
disp('The system is atleast locally stable as it has poles on the LHP');
disp(' ');
