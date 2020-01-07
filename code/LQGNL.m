function [xdd, theta1dd, theta2dd] = LQGNL(F, theta1, theta2, theta1d, theta2d)
M = 1000;
m1 = 100;
m2 = 100;
l1 = 20;
l2 = 10;
g =9.8;
xdd = (F - m1*g*sind(theta1)*cosd(theta1) - m2*g*sind(theta2)*cosd(theta2) - m1*l1*theta1d*theta1d*sind(theta1) - m2*l2*theta2d*theta2d*sind(theta2))/(M + m1*sind(theta1)*sind(theta1) + m2*sind(theta2)*sind(theta2));
theta1dd = (xdd*cosd(theta1) - g*(sind(theta1)))/(l1);
theta2dd = (xdd*cosd(theta2) - g*(sind(theta2)))/(l2);
end