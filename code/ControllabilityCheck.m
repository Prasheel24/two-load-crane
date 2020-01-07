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
D = zeros(3,1);
% D = 0;


%----------- Check Controllability for different conditions ------------%
%----------- For l1 = l2 = k--------%
l1 = k;
l2 = k;
CT1 = subs(B);
CT2 = subs(A*B);
CT3 = subs(A*CT2);
CT4 = subs(A*CT3);
CT5 = subs(A*CT4);
CT6 = subs(A*CT5);
CTCOND1 = [CT1 CT2 CT3 CT4 CT5 CT6];
if length(A) == rank(CTCOND1)
    disp('The system is Controllable for l1 = l2');
else
    disp("System is not Controllable for l1 = l2");
end
%----------- For m1 = m2 = k--------%
m1 = k;
m2 = k;
syms l1;
syms l2;
CTM1 = subs(B);
CTM2 = subs(A*B);
CTM3 = subs(A*CTM2);
CTM4 = subs(A*CTM3);
CTM5 = subs(A*CTM4);
CTM6 = subs(A*CTM5);
CTCOND2 = [CTM1 CTM2 CTM3 CTM4 CTM5 CTM6];
if length(A) == rank(CTCOND2)
    disp('The system is Controllable for m1 = m2');
else
    disp("System is not Controllable for m1 = m2");
end
%-------------- For M = m1 = m2 = k -----------%
M = k;
m1 = k;
m2 = k;
CTMM1 = subs(B);
CTMM2 = subs(A*B);
CTMM3 = subs(A*CTMM2);
CTMM4 = subs(A*CTMM3);
CTMM5 = subs(A*CTMM4);
CTMM6 = subs(A*CTMM5);
CTCOND3 = [CTMM1 CTMM2 CTMM3 CTMM4 CTMM5 CTMM6];
if length(A) == rank(CTCOND3)
    disp('The system is Controllable for M = m1 = m2');
else
    disp("System is not Controllable for M = m1 = m2");
end
disp(' ');
