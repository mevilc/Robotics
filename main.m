
% Translation-only parts of Transformation matrix
TOAe = [eye(3,3); 0 0 0];
TOAe = [TOAe, [4.123, 0, 0, 1]'];

TABe = [eye(3,3); 0 0 0];
TABe = [TABe, [4.123, 0, 0, 1]'];

TBCe = [eye(3,3); 0 0 0];
TBCe = [TBCe, [2.828, 0, 0, 1]'];

TCDe = [eye(3,3); 0 0 0];
TCDe = [TCDe, [10.77, 0, 0, 1]'];

TDEe = [eye(3,3); 0 0 0];
TDEe = [TDEe, [6.325, 0, 0, 1]'];

TEFe = [eye(3,3); 0 0 0];
TEFe = [TEFe, [1, 0, 0, 1]'];


y = ones(1,28800); % array of 1's to be later added with O's
max = 0; % max distance point F is in the x axis wrt to frame O

load encoder.txt;
figure(1);

% Loop through .txt file
for i = 1:28800

    q1 = encoder(i, 1) + 1.81;  % add relative angles (in rad) to encoder readings
    R01 = rotmat(q1); % Rotation mattix from frame O to A
    TOA = [R01; 0 0 0];
    TOA = [TOA, [0; 0; 0; 1]]; % Rotation-only part of Transformation matrix from frame O to A
    
    q2 = encoder(i, 2) - 0.485;
    R12 = rotmat(q2);
    TAB = [R12; 0 0 0];
    TAB = [TAB, [0; 0; 0; 1]];
    
    q3 = encoder(i, 3) + 1.025;
    R23 = rotmat(q3);
    TBC = [R23; 0 0 0];
    TBC = [TBC, [0; 0; 0; 1]];
    
    q4 = encoder(i, 4) - 1.97;
    R34 = rotmat(q4);
    TCD = [R34; 0 0 0];
    TCD = [TCD, [0; 0; 0; 1]];
    
    q5 = encoder(i, 5) + 0.869;
    R45 = rotmat(q5);
    TDE = [R45; 0 0 0];
    TDE = [TDE, [0; 0; 0; 1]];
    
    q6 = encoder(i, 6)-1.249;
    R56 = rotmat(q6);
    TEF = [R56; 0 0 0];
    TEF = [TEF, [0; 0; 0; 1]];
    
    TOEp = TOA*TOAe*TAB*TABe*TBC*TBCe*TCD*TCDe*TDE*TDEe*TEF; % frame O to E'
    TOF = TOEp*TEFe;
 
    pG_F = inv(TOF)*[40; 21; 0; 1]; % point G wrt frame F
    pH_F = inv(TOF)*[40; 19; 0; 1]; % point H wrt frame F
    pF_O = TOEp*[1;0;0;1]; % point F wrt frame O
    
    % update max distance of point F wrt to frame O
    if pF_O(1) >= max
        max = pF_O(1);
    end
    
    % check if points G and H (both wrt frame F) dont fall in the window of +0.5
    % to -0.5 of the origin of frame F. Here, 0.5 is scaled to 15 (unit
    % vector scaling) to reflect realistic distance
    
    if pG_F(2) < -15 && pH_F(2) < pG_F(2)
        y(i) = 0;
    elseif pG_F(2) > pH_F(2) && pH_F(2) > 15
        y(i) = 0;
    end
    
end

plot((1:28800)/3600, y(1:28800)); % x = each sec in terms of hour, y = 1 or 0
axis([-0.5 9 -0.5 1.5]);
ylabel('Binary Signal','fontweight','bold'); 
xlabel('Time (h)');
set(gca, 'xtick', 0:2:10);

fprintf('The minimum distance from the camera to window is: %d m.\n', 40-max); % 40 is distance from frame O origin to G

% calculate Z-rotation matrix with radian angles
function R = rotmat(theta)

R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];

end