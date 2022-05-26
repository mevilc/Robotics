% MEEG 671
% Homework 5
% Mevil Crasta

pd = [0.7458; -0.2494; 0.4810]; % from given T07

% given R07, find euler angles for orientation
phi = atan2(-0.1357,0.8984);
theta = atan2(sqrt(0.8984^2+0.1357^2),0.4177);
psi = atan2(-0.8114,0.4089);
phid = [phi; theta; psi];

steps = 1000000;

q = zeros(7, steps); % hold all angles at each time until e -> 0
e = zeros(6, steps); % desired pose - actual pose. Last col = 0.001

J = zeros(6,7);

% Ta(phi) matrix to go from geometric to analytic Jacobian
Ta = [1 0 0 0 0 0; 
      0 1 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 0 -sin(phi) cos(phi)*sin(theta);
      0 0 0 0 cos(phi) sin(phi)*sin(theta);
      0 0 0 1 0 cos(theta)];

q(:,1) = [pi/7; pi/6; pi/5; pi/4; pi/3; pi/2; pi];
K = 100*eye(6,6);

for i=1:steps
    xe = fwd_kin(q(:,i));
    Ja = Jacobian(q(:,i), pd, J, Ta);
    e(:,i) = [pd; phid] - xe;
    pseudo_inv = pinv(Ja);
    qdot = pseudo_inv*K*e(:,i);
    q(:,i+1) = q(:,i) + qdot*0.001;
    if (max(abs(e(:,i))) < 0.00001)
        final_config = q(:,i); 
        break;
    end
end

% Test by doing forward kinematics on final q
check = fwd_kin(final_config);
disp("Checking with forward kinematics");
disp(check);

% output
T07 = [0.0525 0.4360 0.8984 0.7458; -0.9111 0.3893 -0.1357 -0.2494;
       -0.4089 -0.8114 0.4177 0.4810; 0         0       0       1];
disp("desired transformation matrix");
disp(T07);

disp("computed configuration of each joint angle");
disp(q(:,109));

disp("actual transformation matrix");
disp(final_trans_matrix(final_config));

function Jac = Jacobian(q, pe, J, Ta)

    % extract each angle from q(:,i)
    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5); q6 = q(6);
    
    A01 = [cos(q1) 0 sin(q1) 0; sin(q1) 0 -cos(q1) 0; 0 1 0 0.3105; 0 0 0 1];
    A12 = [cos(q2) 0 -sin(q2) 0; sin(q2) 0 cos(q2) 0; 0 -1 0 0; 0 0 0 1];
    A02 = A01*A12;
    A23 = [cos(q3) 0 -sin(q3) 0; sin(q3) 0 cos(q3) 0; 0 -1 0 0.4; 0 0 0 1];
    A03 = A02*A23;
    A34 = [cos(q4) 0 sin(q4) 0; sin(q4) 0 -cos(q4) 0; 0 1 0 0; 0 0 0 1];
    A04 = A03*A34;
    A45 = [cos(q5) 0 sin(q5) 0; sin(q5) 0 -cos(q5) 0; 0 1 0 0.39; 0 0 0 1];
    A05 = A04*A45;
    A56 = [cos(q6) 0 -sin(q6) 0; sin(q6) 0 cos(q6) 0; 0 -1 0 0; 0 0 0 1];
    A06 = A05*A56;
    
    z0 = [0 0 1]'; p0 = [0 0 0]';
    z1 = A01(1:3,3); p1 = A01(1:3,4);
    z2 = A02(1:3,3); p2 = A02(1:3,4);
    z3 = A03(1:3,3); p3 = A03(1:3,4);
    z4 = A04(1:3,3); p4 = A04(1:3,4);
    z5 = A05(1:3,3); p5 = A05(1:3,4);
    z6 = A06(1:3,3); p6 = A06(1:3,4);
    
    Jg = [cross(z0,pe-p0) cross(z1,pe-p1) cross(z2,pe-p2) cross(z3,pe-p3) cross(z4,pe-p4) cross(z5,pe-p5) cross(z6,pe-p6);
           z0 z1 z2 z3 z4 z5 z6];
    
    Jac = inv(Ta)*Jg;
end

% return FK => [x; y; z; phi; theta; psi];
function a = fwd_kin(q)
    
    T07 = final_trans_matrix(q);
    x = T07(1,4); y = T07(2,4); z = T07(3,4);
    
    % get euler angles from T07
    r23 = T07(2,3); r13 = T07(1,3); r33 = T07(3,3); r32 = T07(3,2); r31 = T07(3,1);
    phi = atan2(r23,r13);
    theta = atan2(sqrt(r13^2+r23^2),r33);
    psi = atan2(r32,-r31);

    a = [x; y; z; phi; theta; psi];
end

function b = final_trans_matrix(q)
    
    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5); q6 = q(6); q7 = q(7);

    A01 = [cos(q1) 0 sin(q1) 0; sin(q1) 0 -cos(q1) 0; 0 1 0 0.3105; 0 0 0 1];
    A12 = [cos(q2) 0 -sin(q2) 0; sin(q2) 0 cos(q2) 0; 0 -1 0 0; 0 0 0 1];
    A02 = A01*A12;
    A23 = [cos(q3) 0 -sin(q3) 0; sin(q3) 0 cos(q3) 0; 0 -1 0 0.4; 0 0 0 1];
    A03 = A02*A23;
    A34 = [cos(q4) 0 sin(q4) 0; sin(q4) 0 -cos(q4) 0; 0 1 0 0; 0 0 0 1];
    A04 = A03*A34;
    A45 = [cos(q5) 0 sin(q5) 0; sin(q5) 0 -cos(q5) 0; 0 1 0 0.39; 0 0 0 1];
    A05 = A04*A45;
    A56 = [cos(q6) 0 -sin(q6) 0; sin(q6) 0 cos(q6) 0; 0 -1 0 0; 0 0 0 1];
    A06 = A05*A56;
    A67 = [cos(q7) -sin(q7) 0 0; sin(q7) cos(q7) 0 0; 0 0 1 0.083; 0 0 0 1];
    T07 = A06*A67;
    b = T07;
end