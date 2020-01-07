%----------- Define same initial Conditions-------------%
m1=100;
m2=100;
M = 1000;
g = 9.8;
l1 =20;
l2 =10; 
wInitial= [3; 0; 0.5; 0; 0.5; 0];
timeStep=0:0.01:100;

[timeStep,w_state]=ode45(@findWState, timeStep, wInitial);
plot(timeStep, w_state, 'linewidth', 2);
title('Nonlinear Response for the System');

function dw_state= findWState(t,w_state)
m1=100;
m2=100;
M = 1000;
g = 9.8;
l1 =20;
l2 =10; 
A_NLI= [0 1 0 0 0 0 ; 0 0 -(m1*g/M) 0 -(m2*g/M) 0 ; 0 0 0 1 0 0 ; 0 0 -(g*(M+m1))/(M*l1) 0 -(m2*g)/(M*l1) 0 ;0 0 0 0 0 1; 0 0 -(m1*g)/(M*l2) 0 -(g*(M+m2))/(M*l2) 0];
B_NLI= [ 0; 1/M ;0; 1/(M*l1) ;0 ;1/(M*l2)];
C = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0;
    ];
D=[0];
R = 0.00001;
QNLI = [1 0 0 0 0 0;
       0 1 0 0 0 0;
       0 0 100 0 0 0; 
       0 0 0 500 0 0; 
       0 0 0 0 250 0; 
       0 0 0 0 0 2000];
K_NLI=lqr(A_NLI,B_NLI,QNLI,R);
eig(A_NLI-B_NLI * K_NLI);
u = -K_NLI * w_state;
dw_state = zeros(6,1);
dw_state(1)= w_state(2);
dw_state(2)= -((-u) + m1*l1*w_state(4)^2*sin(w_state(3)) + m1*g*sin(w_state(3))*cos(w_state(3))+ m2*l2*w_state(6)^2*sin(w_state(5))+m2*g*sin(w_state(5))*cos(w_state(5)))/(M+m1*sin(w_state(3)^2)+ m2*sin(w_state(5)^2));
dw_state(3)= w_state(4);
dw_state(4)= -((-u) +(M+m1)*g*sin(w_state(3)) + m1*l1*w_state(4)^2*sin(w_state(3))*cos(w_state(3)) + m2*l2*w_state(6)^2*sin(w_state(5))*cos(w_state(3)) + m2*g*sin(w_state(5))*cos(w_state(3)-w_state(5)))/(( M+ m1*sin(w_state(3)^2)+ m2*sin(w_state(5)^2))*l1);
dw_state(5)= w_state(6);
dw_state(6)= -((-u) + m1*l1*w_state(4)^2*sin(w_state(3))*cos(w_state(5)) +m1*g*sin(w_state(3))*cos(w_state(3)-w_state(5)) + (M+m1)*g*sin(w_state(5))+m2*l2*w_state(6)^2*sin(w_state(5))*cos(w_state(5)))/(( M+ m1*sin(w_state(3)^2)+ m2*sin(w_state(5)^2))*l2);
end

