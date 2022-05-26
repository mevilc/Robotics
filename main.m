% MEEG 671
% Final Project
% Mevil Crasta

% Distances in m, angles in rad

qc = deg2rad([78.74 18.46 0.35 -65.23 1.43 7.11 -1.19]); % in rad

% transf. matrix from base to end effector at qc
T0E_qc = final_trans_matrix(qc);

% transf. matrix from end effector to camera
Rz = [cosd(-68) -sind(-68) 0; sind(-68) cosd(-68) 0; 0 0 1]; % Rz(-68deg)
Rx = [1 0 0; 0 cosd(-90) -sind(-90); 0 sind(-90) cosd(-90)]; % Rx(-90deg)
REC = Rz*Rx;
TECr = [REC; 0 0 0];
TECr = [TECr, [0; 0; 0; 1]]; % rotation only part
TECe = [eye(3,3); 0 0 0];

% 2.1in = 0.0533m; -1.72in = -0.0437m; 1.55in = 0.0394m
TECe = [TECe, [0.0533; -0.0437; 0.0394; 1]]; % translation only part
TEC = TECr*TECe;

% transf. matrix from camera to Aruco at qc
roll = deg2rad(-177.1661574999052);
pitch = deg2rad(-21.5174231910357);
yaw = deg2rad(-12.2087112074803);
RCA = eul2rotm([roll pitch yaw],'xyz');
TCA = [RCA; 0 0 0];
TCA = [TCA, [0.169432340063091; -0.084399496991377; 0.760080832922170; 1]];

% transf. matrix from Aruco to target
RAT = [-1 0 0; 0 1 0; 0 0 -1];
TATr = [RAT; 0 0 0];
TATr = [TATr, [0; 0; 0; 1]];
TATe = [eye(3,3); 0 0 0];

% -2.3605in = -0.05996m; -4.124in = -0.1047m;
TATe = [TATe, [-0.05996; -0.1047; 0; 1]];
TAT = TATr*TATe;

% transf. matrix from  base to target
T0T = T0E_qc*TEC*TCA*TAT;

% transf. matrix from end effector to object
REobj = [cosd(112) -sind(112) 0; sind(112) cosd(112) 0; 0 0 1]; % Rz(112deg)
TEobjr = [REobj; 0 0 0];
TEobjr = [TEobjr, [0; 0; 0; 1]];
TEobjt = [eye(3,3); 0 0 0];

% -0.1855in = -0.0047m; -0.149in = -0.0038m; 1.403in = 0.0356m;
TEobjt = [TEobjt, [-0.0047; -0.0038; 0.0356; 1]];
TEobj = TEobjr*TEobjt;

% transf. matrix from base to end effector w/ object ==> solving this
T0E_final = T0T*inv(TEobj);

R0E_final = T0E_final(1:3,1:3);
angles = rotm2eul(R0E_final,'zyz');
phi_final = angles(1);
theta_final = angles(2);
psi_final = angles(3);
x_final = T0E_final(1,4);
y_final = T0E_final(2,4);
z_final = T0E_final(3,4);

% desired pose of end effector wrt to base
pd = [x_final; y_final; z_final];
phid = [phi_final; theta_final; psi_final];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% write to txt file
fileID = fopen('Crasta_Mevil.txt','w');
fmt = '%.4f %.4f %.4f %.4f %.4f %.4f %.4f\n';

% inverse kinematics
%q1 = deg2rad([58.2686 75.3224 11.7968 45.9029 -22.1081 -31.2831 -42.3712]); % in rad
q1 = [1.0170 1.3146 0.2059 0.8012 -0.3859 -0.5460 -0.7395];
steps = 1000000;

q = zeros(7, steps); % hold all angles at each time until e -> 0
e = zeros(6, steps); % desired pose - actual pose. Last col = 0.001

J = zeros(6,7);

% Ta(phi) matrix to go from geometric to analytic Jacobian
Ta = [1 0 0 0 0 0; 
      0 1 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 0 -sin(phi_final) cos(phi_final)*sin(theta_final);
      0 0 0 0 cos(phi_final) sin(phi_final)*sin(theta_final);
      0 0 0 1 0 cos(theta_final)];

q(:,1) = q1';
K = 7; %4

for i=1:steps
    xe = fwd_kin(q(:,i));
    Ja = Jacobian(q(:,i), pd, Ta);
    e(:,i) = [pd; phid] - xe;
    pseudo_inv = pinv(Ja);
    qdot = pseudo_inv*K*e(:,i);
    q(:,i+1) = q(:,i) + qdot*0.01; 
    if (max(abs(e(:,i))) < 0.00001) 
        final_config = q(:,i);
        break;
    end 
end


% final_config in [-pi,pi] range
final_config = [0.9381 2.0648 -1.4276 1.0168 1.0710 1.3931 0.4203];

% Design trajectory
t = 0 : 0.005 : 5;

Traj = [0.0013*t.^3 - 0.0095*t.^2 + 1.0170; 
       -0.0120*t.^3 + 0.0900*t.^2 + 1.3146; ...
        0.0261*t.^3 - 0.1960*t.^2 + 0.2059; ...
       -0.0034*t.^3 + 0.0259*t.^2 + 0.8012;
       -0.0233*t.^3 + 0.1748*t.^2 - 0.3859; ...
       -0.0310*t.^3 + 0.2327*t.^2 - 0.5460;
       -0.0186*t.^3 + 0.1392*t.^2 - 0.7395];


fprintf(fileID,fmt,Traj);

% Check if all angles in trajectory are in [-pi,pi]
%{
ct = 0;
for i=1:length(Traj)
    for j=1:7
        if Traj(j,i) > 2*pi || Traj(j,i) < -2*pi
            ct = ct + 1;
        end
    end
end
%}

% obstacle avoidance check - track y and z position of EE

%{
for i=1:length(Traj)
    POSE = un(Traj(:,i));
    J3Y(i) = POSE(2,4);
    J4Y(i) = POSE(6,4);
    J5Y(i) = POSE(10,4);
    J6Y(i) = POSE(14,4);
    J7Y(i) = POSE(18,4);

    J3Z(i) = POSE(3,4);
    J4Z(i) = POSE(7,4);
    J5Z(i) = POSE(11,4);
    J6Z(i) = POSE(15,4);
    J7Z(i) = POSE(19,4);
end



figure(1);
hold on
plot(t,J3Y);
plot(t,J4Y);
plot(t,J5Y);
plot(t,J6Y);
plot(t,J7Y);
%plot(J7Y, J7Z);
legend('Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'EE');
hold off


figure(2);
hold on
plot(t,J3Z);
plot(t,J4Z);
plot(t,J5Z);
plot(t,J6Z);
plot(t,J7Y);
legend('Joint 3', 'Joint 4', 'Joint 5', 'Joint 6','EE');
hold off
%}

% Check for position and velocity violations - None

%{
Traj = rad2deg(Traj);
ct = 0;

for i=1:length(Traj)-1
    for j=1:7
        if j==1 || j==3 || j==5
            if Traj(j,i) >= 170 && Traj(j,i) <= -170
                ct = ct + 1; 
            end
        elseif j==2 || j==4 || j==6
            if Traj(j,i) >= 120 && Traj(j,i) <= -120
                ct = ct + 1; 
            end
        elseif j==7
            if Traj(j,i) >= 175 && Traj(j,i) <= -175
                ct = ct + 1; 
            end
        end
    end
end
%}

%{

t1 = rad2deg(gradient(0.0013*t.^3 - 0.0095*t.^2 + 1.0710));
t2 = rad2deg(gradient(-0.0120*t.^3 + 0.0900*t.^2 + 1.3146));
t3 = rad2deg(gradient(0.0261*t.^3 - 0.1960*t.^2 + 0.2059));
t4 = rad2deg(gradient(-0.0035*t.^3 + 0.0259*t.^2 + 0.8012));
t5 = rad2deg(gradient(-0.0233*t.^3 + 0.1748*t.^2 - 0.3859));
t6 = rad2deg(gradient(-0.0310*t.^3 + 0.2327*t.^2 - 0.5460));
t7 = rad2deg(gradient(-0.0186*t.^3 + 0.1392*t.^2 - 0.7395));

ct = 0;

for i=1:length(t7) 
    if t7(i)/0.005 >= 180 || t7(i)/0.005 <= -180
        ct = ct + 1;
    end 
end
%}


