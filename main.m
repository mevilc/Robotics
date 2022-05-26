% MEEG 671 
% Homework 2
% Mevil Crasta

a1 = 0.5; a2 = 0.3; a3 = 0.2;

% Part a
q1m = -pi/3; q1M = pi/3;
q2m = -2*pi/3; q2M = 2*pi/3;
q3m = -pi/2; q3M = pi/2;

figure(1)
subplot(1,2,1);
hold on

arc(0,0,a1,q1m,q1M);
arc(0,0,a1+a2,q1m,q1M);
arc(0,0,a1+a2+a3,q1m,q1M);

arc(a1*cos(q1m),a1*sin(q1m),a2,q1m+q2m,q1m+q2M);
arc(a1*cos(q1M), a1*sin(q1M), a2, q2m+q1M, q2M+q1M);

arc((a1+a2)*cos(q1m), (a1+a2)*sin(q1m), a3, q3m+q1m, q3M+q1m);
arc((a1+a2)*cos(q1M), (a1+a2)*sin(q1M), a3, q3m+q1M, q3M+q1M);
arc(a1*cos(q1M)+a2*cos(q1M+q2M), a1*sin(q1M)+a2*sin(q1M+q2M), ...
    a3, q3m+q1M+q2M, q3M+q1M+q2M);
arc(a1*cos(q1M)+a2*cos(q1M+q2m), a1*sin(q1M)+a2*sin(q1M+q2m), ...
    a3, q3m+q1M+q2m, q3M+q1M+q2m);
arc(a1*cos(q1m)+a2*cos(q1m+q2M), a1*sin(q1m)+a2*sin(q1m+q2M), ...
    a3, q3m+q1m+q2M, q3M+q1m+q2M);
arc(a1*cos(q1m)+a2*cos(q1m+q2m), a1*sin(q1m)+a2*sin(q1m+q2m), ...
    a3, q3m+q1m+q2m, q3M+q1m+q2m);

arc(a1*cos(q1M), a1*sin(q1M), a2+a3, q1M+q2m, q1M+q2M);
arc(a1*cos(q1m), a1*sin(q1m), a2+a3, q1m+q2m, q1m+q2M);
arc(a1+a2,0,a3,q3m,q3M);
arc(a1,0,a2+a3,q2m,q2M);
arc(a1+a2*cos(q2M),a2*sin(q2M), a3, q2M+q3m, q2M+q3M);
arc(a1+a2*cos(q2m),a2*sin(q2m), a3, q2m+q3m, q2m+q3M);

arc(0, 0, sqrt( (a1*cos(q1m)+a2*cos(q1m+q2m)+a3*cos(q1m+q2m+q3m))^2 + ...
    (a1*sin(q1m)+a2*sin(q1m+q2m)+a3*sin(q1m+q2m+q3m))^2 ), ...
    atan2(a1*sin(q1m)+a2*sin(q1m+q2m)+a3*sin(q1m+q2m+q3m), ...
    a1*cos(q1m)+a2*cos(q1m+q2m)+a3*cos(q1m+q2m+q3m)), ...
    -atan2(a1*sin(q1m)+a2*sin(q1m+q2m)+a3*sin(q1m+q2m+q3m),a1*cos(q1m)+a2*cos(q1m+q2m)+a3*cos(q1m+q2m+q3m)));

arc(0, 0, sqrt( (a1*cos(q1m)+a2*cos(q1m+q2m)+a3*cos(q1m+q2m+q3M))^2 + ...
    (a1*sin(q1m)+a2*sin(q1m+q2m)+a3*sin(q1m+q2m+q3M))^2 ), ...
    atan2(a1*sin(q1m)+a2*sin(q1m+q2m)+a3*sin(q1m+q2m+q3M),a1*cos(q1m)+a2*cos(q1m+q2m)+a3*cos(q1m+q2m+q3M)), ...
    -atan2(a1*sin(q1m)+a2*sin(q1m+q2m)+a3*sin(q1m+q2m+q3M),a1*cos(q1m)+a2*cos(q1m+q2m)+a3*cos(q1m+q2m+q3M)));

arc(0, 0, sqrt( (a1*cos(q1m)+a2*cos(q1m+q2m))^2 + (a1*sin(q1m)+a2*sin(q1m+q2m))^2), ...
    atan2(a1*sin(q1m)+a2*sin(q1m+q2m),a1*cos(q1m)+a2*cos(q1m+q2m)), ...
    -atan2(a1*sin(q1m)+a2*sin(q1m+q2m),a1*cos(q1m)+a2*cos(q1m+q2m)));

arc(0, 0, sqrt( ((a1+a2)*cos(q1m)+a3*cos(q1m+q3m))^2 + ((a1+a2)*sin(q1m)+a3*sin(q1m+q3m))^2 ), ...
    atan2((a1+a2)*sin(q1m)+a3*sin(q1m+q3m),(a1+a2)*cos(q1m)+a3*cos(q1m+q3m)), ...
    -atan2((a1+a2)*sin(q1m)+a3*sin(q1m+q3m),(a1+a2)*cos(q1m)+a3*cos(q1m+q3m)));
hold off

% Part b
t1 = q1m:0.1:q1M;
t2 = q2m:0.1:q2M; 
t3 = q3m:0.1:q3M;

[T1,T2, T3] = meshgrid(t1,t2,t3); 

x = a1 * cos(T1) + a2 * cos(T1 + T2) + a3 * cos(T1 + T2 + T3); 
y = a1 * sin(T1) + a2 * sin(T1 + T2) + a3 * sin(T1 + T2 + T3);

subplot(1,2,2);
plot(x(:),y(:),'r.'); 
axis equal;
title('Validation Plot');

% Function to draw arc for part a
function arc(x0, y0, r, T1, T2)
    theta = T1 : 0.1 : T2;
    x = x0 + r * cos(theta);
    y = y0 + r * sin(theta);
    plot(x, y,'black');
    title('Robot Workspace');
    axis equal;
end