% % Motion Planning - Cartesian Space
function motion_planning_cartesian(A,B,C)

sampling_rate = 0.002;
A(1:3,4) = A(1:3,4)/100;
B(1:3,4) = B(1:3,4)/100;
C(1:3,4) = C(1:3,4)/100;

A_pos = [A(1,4) A(2,4) A(3,4) matrixtoangle(A(1:3,1:3))];
B_pos = [B(1,4) B(2,4) B(3,4) matrixtoangle(B(1:3,1:3))];
C_pos = [C(1,4) C(2,4) C(3,4) matrixtoangle(C(1:3,1:3))];


delta_b = B_pos - A_pos;
delta_c = C_pos - B_pos;

% although the textbook make motion planning in -0.5~0.5s, but it will
% cause a little problem in t, so I choose 0 ~ 1s

Tacc = 0.2;
t0 = 0.0;
t1 = 0.5 - Tacc;
t2 = 0.5;
t3 = 0.5 + Tacc;
t4 = 1.0;

pos = zeros(6,1/sampling_rate);
vel = zeros(6,1/sampling_rate);
acc = zeros(6,1/sampling_rate);

%% Start Motion Planning - follow textbook
i = 1;
for t = t0:sampling_rate:t1 - sampling_rate
    h = t/t2;
    pos(:,i) = delta_b*h + A_pos;
    vel(:,i) = delta_b/t2;
    acc(:,i) = 0;
    i = i+1;
end

for t = t1:sampling_rate:t3-sampling_rate
    h = (t-t1)/(t3-t1);
    pos(:,i) = ((delta_c*Tacc/t2 - delta_b*Tacc/t2)*(2-h)*h^2 + 2*delta_b*Tacc/t2)*h + B_pos - delta_b*Tacc/t2;
    vel(:,i) = ((delta_c*Tacc/t2 - delta_b*Tacc/t2)*(1.5-h)*2*h^2 + delta_b*Tacc/t2) * (1/Tacc);
    acc(:,i) = (delta_c*Tacc/t2 - delta_b*Tacc/t2) * (1-h) * (3*h/(Tacc^2));
    i = i+1;
end

for t = t3:sampling_rate:t4
    h = (t-t2)/(t4-t2);
    pos(:,i) = delta_c*h + B_pos;
    vel(:,i) = delta_c/(t4 - t2);
    acc(:,i) = 0;
    i = i+1;
end

%% Turn to angle by IK
joint_matrix = zeros(4,4);
joint_angle = zeros(6,1/sampling_rate);

for i = 1:length(pos)
    joint_matrix(1:3,1:3) = angletomatrix(pos(4:6,i)); 
    joint_matrix(1:3,4) = pos(1:3,i);
    joint_angle(:,i) = inverse_kinematics(joint_matrix);
end

joint_vel = diff(joint_angle')';
joint_acc = diff(joint_vel')';

p(:,:) = pos(1:3,:);
v(:,:) = vel(1:3,:);
a(:,:) = acc(1:3,:);

%% show the angle , angular velocity, angular accleration
t = -0.5:sampling_rate:0.5;

draw_joint(joint_angle,joint_vel,joint_acc,2)

%% show the position, velocity, acceleration of x,y,z
figure

subplot(331), plot(t,p(1,:))
title('Position of x')
subplot(332), plot(t,v(1,:))
title('Velocity of x')
subplot(333), plot(t,a(1,:))
title('Acceleration of x')

subplot(334), plot(t,p(2,:))
title('Position of y'), ylabel('Position(cm)')
subplot(335), plot(t,v(2,:))
title('Velocity of y'), ylabel('Velocity(cm/s)')
subplot(336), plot(t,a(2,:))
title('Acceleration of y'), ylabel('Acceleration(cm/s^2)')

subplot(337), plot(t,p(3,:))
title('Position of z'), xlabel('Time(s)')
subplot(338), plot(t,v(3,:))
title('Velocity of z'), xlabel('Time(s)')
subplot(339), plot(t,a(3,:))
title('Acceleration of z'), xlabel('Time(s)')

%% Show the path
figure
% show the moving path
plot3(p(1,:), p(2,:), p(3,:))
hold on

% show the A B C and their orientation
Ax = A*[0.1 0 0 1]';
Ay = A*[0 0.1 0 1]';
Az = A*[0 0 0.1 1]';
plot3([A(1,4) Ax(1)], [A(2,4) Ax(2)], [A(3,4) Ax(3)], 'r', 'LineWidth', 2)
plot3([A(1,4) Ay(1)], [A(2,4) Ay(2)], [A(3,4) Ay(3)], 'g', 'LineWidth', 2)
plot3([A(1,4) Az(1)], [A(2,4) Az(2)], [A(3,4) Az(3)], 'b', 'LineWidth', 2)
Bx = B*[0.1 0 0 1]';
By = B*[0 0.1 0 1]';
Bz = B*[0 0 0.1 1]';
plot3([B(1,4) Bx(1)], [B(2,4) Bx(2)], [B(3,4) Bx(3)], 'r', 'LineWidth', 2)
plot3([B(1,4) By(1)], [B(2,4) By(2)], [B(3,4) By(3)], 'g', 'LineWidth', 2)
plot3([B(1,4) Bz(1)], [B(2,4) Bz(2)], [B(3,4) Bz(3)], 'b', 'LineWidth', 2)
Cx = C*[0.1 0 0 1]';
Cy = C*[0 0.1 0 1]';
Cz = C*[0 0 0.1 1]';
plot3([C(1,4) Cx(1)], [C(2,4) Cx(2)], [C(3,4) Cx(3)], 'r', 'LineWidth', 2)
plot3([C(1,4) Cy(1)], [C(2,4) Cy(2)], [C(3,4) Cy(3)], 'g', 'LineWidth', 2)
plot3([C(1,4) Cz(1)], [C(2,4) Cz(2)], [C(3,4) Cz(3)], 'b', 'LineWidth', 2)

scatter3(A(1,4), A(2,4), A(3,4), [], 'k', 'filled')
text(A(1,4)-0.01, A(2,4)-0.01, A(3,4)-0.01, 'A(40,-30,10)')
scatter3(B(1,4), B(2,4), B(3,4), [], 'k', 'filled')
text(B(1,4)-0.01, B(2,4)-0.01, B(3,4)-0.01, 'B(30,30,20)')
scatter3(C(1,4), C(2,4), C(3,4), [], 'k', 'filled')
text(C(1,4)-0.01, C(2,4)-0.01, C(3,4)-0.01, 'C(40,20,-30)')

%畫出虛線
hold on;
plot3([A(1,4),B(1,4)], [A(2,4),B(2,4)], [A(3,4),B(3,4)], ':');
hold on;
plot3([C(1,4),B(1,4)], [C(2,4),B(2,4)], [C(3,4),B(3,4)], ':');

% path
for i = 1:15:length(pos)
    matrix = angletomatrix([pos(4,i),pos(5,i),pos(6,i)]);
    quiver3(pos(1,i),pos(2,i),pos(3,i),matrix(1,1)/20,matrix(2,1)/20,matrix(3,1)/20,'r',"LineWidth",0.3); 
    quiver3(pos(1,i),pos(2,i),pos(3,i),matrix(1,2)/20,matrix(2,2)/20,matrix(3,2)/20,'g',"LineWidth",0.3); 
    quiver3(pos(1,i),pos(2,i),pos(3,i),matrix(1,3)/20,matrix(2,3)/20,matrix(3,3)/20,'b',"LineWidth",0.3); 
end


xlabel('x(cm)'), ylabel('y(cm)'), zlabel('z(cm)')
title('Motion Planning - Cartesian')
axis equal
set(gca,'XGrid','on'), set(gca,'YGrid','on'), set(gca,'ZGrid','on');
hold off

