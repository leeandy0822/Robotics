% Motion Planning - Joint Space
function motion_planning_joint(A,B,C,M_A,M_B,M_C)

Tacc = 0.2;
t0 = 0.0;
t1 = 0.5-Tacc;
t2 = 0.5;
t3 = 0.5+Tacc;
t4 = 1.0;


sampling_rate = 0.002;

%% Motion Planning  -  Follow the textbook
dB = B - A;
dC = C - B;

angle = zeros(6,ceil(1/sampling_rate));
ang_vel = zeros(6,ceil(1/sampling_rate));
ang_acc = zeros(6,ceil(1/sampling_rate));
i = 1;
%% Linear
for t = t0:sampling_rate:t1-sampling_rate
h = t/t2;
angle(:,i) = dB*h + A;
ang_vel(:,i) = dB/t2;
ang_acc(:,i) = 0;
i = i+1;
end
%% Polynomial
for t= t1:sampling_rate:t3-sampling_rate
h = (t-t1)/(t3-t1);
angle(:,i) = ((dC*Tacc/t2-dB*Tacc/t2)*(2-h)*h^2 + 2*dB*Tacc/t2)*h + B - dB*Tacc/t2;
ang_vel(:,i) = ((dC*Tacc/t2-dB*Tacc/t2)*(1.5-h)*2*h^2 + dB*Tacc/t2) * (1/Tacc);
ang_acc(:,i) = (dC*Tacc/t2-dB*Tacc/t2) * (1-h) * (3*h/(Tacc^2));
i = i+1;
end
%% Linear
for t= t3:sampling_rate:t4
h = (t-t2)/(t4-t2);
angle(:,i) = dC*h + B;
ang_vel(:,i) = dC/(t4-t2);
ang_acc(:,i) = 0;
i = i+1;
end

%% Turn to Position, Draw
position  = zeros(3,length(angle));

for i = 1:1:length(angle)
six_theta = angle(:,i)';
[~, p] = forward_kinematics(six_theta);
position(1:3,i) = p(1:3);
end

x = position(1,:);
y = position(2,:);
z = position(3,:);
posvecacc_draw(x,y,z)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Draw the path and the A, B, C point and 虛線 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


%% Draw Joint angle
draw_joint(angle,ang_vel,ang_acc,1)


%% show the A B C and their orientation
figure(10)
plot3(x,y,z);
hold on;
Ax = M_A*[10 0 0 1]';
Ay = M_A*[0 10 0 1]';
Az = M_A*[0 0 10 1]';
plot3([M_A(1,4) Ax(1)], [M_A(2,4) Ax(2)], [M_A(3,4) Ax(3)], 'r', 'LineWidth', 2)
plot3([M_A(1,4) Ay(1)], [M_A(2,4) Ay(2)], [M_A(3,4) Ay(3)], 'g', 'LineWidth', 2)
plot3([M_A(1,4) Az(1)], [M_A(2,4) Az(2)], [M_A(3,4) Az(3)], 'b', 'LineWidth', 2)
Bx = M_B*[10 0 0 1]';
By = M_B*[0 10 0 1]';
Bz = M_B*[0 0 10 1]';
plot3([M_B(1,4) Bx(1)], [M_B(2,4) Bx(2)], [M_B(3,4) Bx(3)], 'r', 'LineWidth', 2)
plot3([M_B(1,4) By(1)], [M_B(2,4) By(2)], [M_B(3,4) By(3)], 'g', 'LineWidth', 2)
plot3([M_B(1,4) Bz(1)], [M_B(2,4) Bz(2)], [M_B(3,4) Bz(3)], 'b', 'LineWidth', 2)
Cx = M_C*[10 0 0 1]';
Cy = M_C*[0 10 0 1]';
Cz = M_C*[0 0 10 1]';
plot3([M_C(1,4) Cx(1)], [M_C(2,4) Cx(2)], [M_C(3,4) Cx(3)], 'r', 'LineWidth', 2)
plot3([M_C(1,4) Cy(1)], [M_C(2,4) Cy(2)], [M_C(3,4) Cy(3)], 'g', 'LineWidth', 2)
plot3([M_C(1,4) Cz(1)], [M_C(2,4) Cz(2)], [M_C(3,4) Cz(3)], 'b', 'LineWidth', 2)

scatter3(M_A(1,4), M_A(2,4), M_A(3,4), [], 'k', 'filled')
text(M_A(1,4)-0.01, M_A(2,4)-0.01, M_A(3,4)-0.01, 'A(40,-30,10)')
scatter3(M_B(1,4), M_B(2,4), M_B(3,4), [], 'k', 'filled')
text(M_B(1,4)-0.01, M_B(2,4)-0.01, M_B(3,4)-0.01, 'B(30,30,20)')
scatter3(M_C(1,4), M_C(2,4), M_C(3,4), [], 'k', 'filled')
text(M_C(1,4)-0.01, M_C(2,4)-0.01, M_C(3,4)-0.01, 'C(40,20,-30)')


%畫出虛線
hold on;
plot3([M_A(1,4),M_B(1,4)], [M_A(2,4),M_B(2,4)], [M_A(3,4),M_B(3,4)], ':');
hold on;
plot3([M_C(1,4),M_B(1,4)], [M_C(2,4),M_B(2,4)], [M_C(3,4),M_B(3,4)], ':');
hold on;

% path
for i = 1:1:length(position)
[P, ~] = forward_kinematics(angle(:,i)');
quiver3(P(1,4),P(2,4),P(3,4),P(1,3)*5,P(2,3)*5,P(3,3)*5,'b',"LineWidth",0.3); hold on
end


xlabel('x(cm)'), ylabel('y(cm)'), zlabel('z(cm)')
title('Motion Planning - Joint')
axis equal
set(gca,'XGrid','on'), set(gca,'YGrid','on'), set(gca,'ZGrid','on');
hold off


