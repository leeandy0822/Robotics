% This function is a FK function
% forward kinematics
function [T6,p] = forward_kinematics(joint_varaible);

angle_to_rad = pi/180;  %transfer angle to rad
rad_to_angle = 180/pi;  %transfer rad to angle

%kinematic table parameter
a1 = 12;    
a2 = 25;     
a3 = 26;

%Forward Kinematic
joint = joint_varaible;
theta1=joint(1,1)*angle_to_rad;
theta2=joint(1,2)*angle_to_rad;
theta3=joint(1,3)*angle_to_rad;
theta4=joint(1,4)*angle_to_rad;
theta5=joint(1,5)*angle_to_rad;
theta6=joint(1,6)*angle_to_rad;

% Transformation matrix A1 - A6
A1 = [ cos(theta1)              0    -sin(theta1)   a1*cos(theta1);
       sin(theta1)              0     cos(theta1)   a1*sin(theta1);
                 0             -1              0                 0;
                 0              0              0                 1  ];
             
A2 = [ cos(theta2)   -sin(theta2)              0    a2*cos(theta2);
       sin(theta2)    cos(theta2)              0    a2*sin(theta2);
                 0              0              1                 0;
                 0              0              0                 1  ];

A3 = [ cos(theta3)   -sin(theta3)              0    a3*cos(theta3);
       sin(theta3)    cos(theta3)              0    a3*sin(theta3);
                 0              0              1                 0;
                 0              0              0                 1  ];
             
A4 = [ cos(theta4)              0    -sin(theta4)                0;
       sin(theta4)              0     cos(theta4)                0;
                 0             -1              0                 0;
                 0              0              0                 1  ];
             
A5 = [ cos(theta5)              0     sin(theta5)                0;
       sin(theta5)              0    -cos(theta5)                0;
                 0              1              0                 0;
                 0              0              0                 1  ];
             
A6 = [ cos(theta6)   -sin(theta6)              0                 0;
       sin(theta6)    cos(theta6)              0                 0;
                 0              0              1                 0;
                 0              0              0                 1  ];

    % calculate final transfermation matrix
    T6=A1*A2*A3*A4*A5*A6;
    
    % get n o a p value
    nx = T6(1,1);
    ny = T6(2,1);
    nz = T6(3,1);
    
    ox = T6(1,2);
    oy = T6(2,2);
    oz = T6(3,2);
    
    ax = T6(1,3);
    ay = T6(2,3);
    az = T6(3,3);
    
    px = T6(1,4);
    py = T6(2,4);
    pz = T6(3,4);
        
    % transfer n o a to phi theta psi
    phi=(atan2(ay,ax)+pi)*rad_to_angle;               
    theta=atan2(sqrt((ax)^2+(ay)^2),az)*rad_to_angle;
    psi=atan2(oz,-nz)*rad_to_angle;

p = [px, py, pz, phi, theta, psi]';
end
