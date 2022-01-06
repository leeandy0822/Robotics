% Draw angle, omega, alpha
function draw_joint(dtheta,domega,dalpha,mode)

sampling_rate = 0.002;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Different motion planning ways will cause different result
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

if mode == 1
angle_limit = 0.5;
vel_limit = 0.5;
acc_limit = 0.5;
else
angle_limit = 0.5;
vel_limit = 0.5 - sampling_rate;
acc_limit = 0.5 - sampling_rate*2;
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Angle
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
figure
t=-0.5:sampling_rate:angle_limit;
for i=1:1:6
    theta = dtheta(i,:); 
    subplot(3,2,i);
    plot(t,theta);
    grid
    title(sprintf('joint%i',i));
    if i==3
        ylabel({'Angle(°)';' '}, 'FontSize', 12, 'FontWeight', 'bold');
    end
end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Angular Velocity
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
t = -0.5:sampling_rate:vel_limit;
figure
for i=1:1:6
    omega = domega(i,:); 
    subplot(3,2,i);
    plot(t,omega);
    grid
    title(sprintf('joint%i',i));
    if i==3
        ylabel({'Angular Velocity(°/s)';' '}, 'FontSize', 12, 'FontWeight', 'bold');
    end
end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Angular Acceleration
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
t = -0.5:sampling_rate:acc_limit;
figure
for i=1:1:6
    alpha = dalpha(i,:); 
    subplot(3,2,i);
    plot(t,alpha);
    grid
    title(sprintf('joint%i',i));
    if i==3
        ylabel({'Angular Acceleration(°/s^2)';' '}, 'FontSize', 12, 'FontWeight', 'bold');
    end
end