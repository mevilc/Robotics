% MEEG 671
% HW7
% Mevil Crasta

                            % TAKES AROUND 1 M 20s TO RUN

u = importdata('torques.txt');

IC1 = [0; 0; 0; 0]; % q1(0) = q(2) = 0; q1dot(0) = q2dot(0) = 0;
IC2 = [0; 0; 0; 0]; % q1(0) = q(2) = 0; q1dot(0) = q2dot(0) = 0;

options = odeset('RelTol',1e-4,'AbsTol',[1e-5 1e-5 1e-5 1e-5]);
qs = zeros(length(u),2);
qd = zeros(length(u),2);

for i=1:length(u)-1
    t1 = [0 0.001];
    torque = u(i,:);
    [tsim,qa] = ode45(@samplesys,t1, IC1, options, torque');
    IC1 = qa(end,:);  
    qs(i+1,:) = qa(end,1:2);

    t2 = [0 0.005];
    [tsim,qb] = ode45(@samplesys,t2, IC2, options, torque');
    IC2 = qb(end,:);  
    qd(i+1,:) = qb(end,1:2);
end   

x_s = zeros(length(qs),1);
y_s = zeros(length(qs),1);
for i=1:length(qs)
    q1 = qs(i,1); q2 = qs(i,2);
    x_s(i) = 0.5*cos(q1)+0.6*cos(q2+q1);
    y_s(i) = 0.5*sin(q1)+0.6*sin(q2+q1);
end

x_d = zeros(length(qd),1);
y_d = zeros(length(qd),1);
for i=1:length(qd)
    q1 = qd(i,1); q2 = qd(i,2);
    x_d(i) = 0.5*cos(q1)+0.6*cos(q2+q1);
    y_d(i) = 0.5*sin(q1)+0.6*sin(q2+q1);
end

figure(1);
plot(x_s,y_s);
title('X-Y Trajectory at 1 KHz');
ylabel('y (m)');
xlabel('x (m)');

figure(2);
plot(x_d,y_d);
title('X-Y Trajectory at 200 Hz');
ylabel('y (m)');
xlabel('x (m)');


function dx = samplesys(t,x,torque)
    [B, C, G] = dynamics_matrices([x(1) x(2)], [x(3) x(4)]);
    
    dx = zeros(4,1);
    q_dot_dot = B \ ( torque - C * [x(3); x(4)] - G );
    dx(1) = x(3);
    dx(2) = x(4);
    dx(3) = q_dot_dot(1);
    dx(4) = q_dot_dot(2);
end
