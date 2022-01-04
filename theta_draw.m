function theta_draw(X,Y,Z)
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    % 從 A 到 C 之 x, y, z 各軸的變化情形 %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    sampling_rate = 0.002;
    t=-0.5:sampling_rate:0.5;
    
    figure(8)
    subplot(3,1,1);
    plot(t,X);
    title('position of x');
    grid
    
    subplot(3,1,2);
    plot(t,Y);
    ylabel({'Position(cm)';' '}, 'FontSize', 12, 'FontWeight', 'bold');
    title('position of y');
    grid
    
    subplot(3,1,3);
    plot(t,Z);
    title('position of z');
    xlabel('Time(s)')
    grid
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    % 從 A 到 C 之 x, y, z 各速度的變化情形 %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    dt=t(2:501);
    dX=diff(X)/sampling_rate;
    dY=diff(Y)/sampling_rate;
    dZ=diff(Z)/sampling_rate;
    
    figure(9)
    subplot(3,1,1);
    plot(dt,dX);
    title('velocity of x');
    grid
    
    subplot(3,1,2);
    plot(dt,dY);
    title('velocity of y');
    ylabel({'Velocity(cm/s)';' '}, 'FontSize', 12, 'FontWeight', 'bold');
    grid
    
    subplot(3,1,3);
    plot(dt,dZ);
    title('velocity of z');
    xlabel('Time(s)')
    grid
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    % 從 A 到 C 之 x, y, z 各加速度的變化情形 %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    dt2=t(3:501);
    dX2=diff(dX)/sampling_rate;
    dY2=diff(dY)/sampling_rate;
    dZ2=diff(dZ)/sampling_rate;
    
    figure(10)
    subplot(3,1,1);
    plot(dt2,dX2);
    title('acceleration of x');
    grid
    
    subplot(3,1,2);
    plot(dt2,dY2);
    title('acceleration of y');
    ylabel({'Acceleration(cm/s^2)';' '}, 'FontSize', 12, 'FontWeight', 'bold');
    grid
    
    subplot(3,1,3);
    plot(dt2,dZ2);
    title('acceleration of z');
    xlabel('Time(s)')
    grid