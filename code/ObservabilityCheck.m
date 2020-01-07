%------------- Linearised System Matrices ---------------%
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

%------------- Observability for x(t) ---------------%
CX = [1 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0;
    ];
%For the system to be controllable, the rank must be equal to matrix A rank
if length(ALI) == rank(obsv(ALI, CX))
    disp('The system is Observable for x(t)');
else
    disp("System is not Observable for x(t)");
end
%------------- Observability for theta1(t), theta2(t) ---------------%
CT12 = [0 0 0 0 0 0;
        0 0 1 0 0 0;
        0 0 0 0 1 0;
    ];
%For the system to be controllable, the rank must be equal to matrix A rank
if length(ALI) == rank(obsv(ALI, CT12))
    disp('The system is Observable for theta1(t) and theta2(t)');
else
    disp("System is not Observable for theta1(t) and theta2(t)");
end
%------------- Observability for x(t), theta2(t) ---------------%
CXT1 = [1 0 0 0 0 0;
        0 0 0 0 0 0;
        0 0 0 0 1 0;
    ];
%For the system to be controllable, the rank must be equal to matrix A rank
if length(ALI) == rank(obsv(ALI, CXT1))
    disp('The system is Observable for x(t) and theta1(t)');
else
    disp("System is not Observable for x(t) and theta1(t)");
end
%------------- Observability for x(t), theta1(t), theta2(t) ------------%
CXT12 = [1 0 0 0 0 0;
         0 0 1 0 0 0;
         0 0 0 0 1 0;
    ];
%For the system to be controllable, the rank must be equal to matrix A rank
if length(ALI) == rank(obsv(ALI, CXT12))
    disp('The system is Observable for x(t), theta1(t) and theta2(t)');
else
    disp("System is not Observable for x(t), theta1(t) and theta2(t)");
end
disp(' ');




%------------- Luenberger observer ------------%
% [LO, PO, EO] = lqe(ALI,BLI,CX,QLI',R);
