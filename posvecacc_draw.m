function posvecacc_draw(X,Y,Z)
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    % 從 A 到 C 之 x, y, z 各軸的變化情形 %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    sampling_rate = 0.002;
    t=-0.5:sampling_rate:0.5;

    dt=t(2:501);
    dX=diff(X)/sampling_rate;
    dY=diff(Y)/sampling_rate;
    dZ=diff(Z)/sampling_rate;
    dt2=t(3:501);
    dX2=diff(dX)/sampling_rate;
    dY2=diff(dY)/sampling_rate;
    dZ2=diff(dZ)/sampling_rate;
    
    figure
    subplot(3,3,1);
    plot(t,X);
    title('position of x');
    grid
    
    subplot(3,3,2);

    plot(dt,dX);
    title('velocity of x');
    grid
    
    subplot(3,3,3);
    plot(dt2,dX2);
    title('acceleration of x');

    xlabel('Time(s)')
    grid
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    % 從 A 到 C 之 x, y, z 各速度的變化情形 %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

    

    subplot(3,3,4);
    plot(t,Y);
    title('position of y'),ylabel('Position(cm)');
    grid
    
    subplot(3,3,5);
    plot(dt,dY);
    title('velocity of y'),ylabel('Velocity(cm/s)');
    grid
    
    subplot(3,3,6);
    plot(dt2,dY2);
    title('acceleration of y'),ylabel('Acceleration(cm/s)');
    xlabel('Time(s)')
    grid
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    % 從 A 到 C 之 x, y, z 各加速度的變化情形 %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


    subplot(3,3,7);
    plot(t,Z);
    title('position of z');
    grid
    
    subplot(3,3,8);


    plot(dt,dZ);
    title('velocity of z');
    grid
    
    subplot(3,3,9);
    plot(dt2,dZ2);
    title('acceleration of z');

    xlabel('Time(s)')
    grid