function theta_correct = inverse_kinematics(NOAP)
%inverse kinematics

% To speed up, this IK only solve 4 solutions
nx=NOAP(1,1);
ny=NOAP(2,1);
nz=NOAP(3,1);
ox=NOAP(1,2);
oy=NOAP(2,2);
oz=NOAP(3,2);
ax=NOAP(1,3);
ay=NOAP(2,3);
az=NOAP(3,3);
px=NOAP(1,4);
py=NOAP(2,4);
pz=NOAP(3,4);

a1 = 0.12;
a2 = 0.25;
a3 = 0.26;

% Theta 1
theta1_1= atan2(py,px);

% theta3 ,four solutions
gamma=cos(theta1_1)*px+sin(theta1_1)*py-a1;
cos31=(gamma^2+pz^2-a2^2-a3^2)/(2*a2*a3);

if 1-cos31^2<0
    theta1_1 = 123456;
    theta3_1 = pi/2;
    theta3_2 = pi/2;
else
    theta3_1=atan2(sqrt(1-cos31^2),cos31);
    theta3_2=atan2(-sqrt(1-cos31^2),cos31);
end

p1=[gamma -pz
    -pz -gamma];

k1=[a3+a2*cos(theta3_1) -a2*sin(theta3_1)]';
r1=p1\k1;
theta23_1=atan2(r1(2,1),r1(1,1));
theta2_1=theta23_1-theta3_1;
k2=[a3+a2*cos(theta3_2) -a2*sin(theta3_2)]';
r2=p1\k2;
theta23_2=atan2(r2(2,1),r2(1,1));
theta2_2=theta23_2-theta3_2;

%define c23 ,s23
s23_1=r1(2,1);
s23_2=r2(2,1);
c23_1=r1(1,1);
c23_2=r2(1,1);

%theta4
g1_1=cos(theta1_1)*c23_1*ax+sin(theta1_1)*c23_1*ay-s23_1*az;
p1_1=-cos(theta1_1)*s23_1*ax-sin(theta1_1)*s23_1*ay-c23_1*az;
theta4_1=atan2(p1_1,g1_1);
theta4_2=atan2(-p1_1,-g1_1);
g1_2=cos(theta1_1)*c23_2*ax+sin(theta1_1)*c23_2*ay-s23_2*az;
p1_2=-cos(theta1_1)*s23_2*ax-sin(theta1_1)*s23_2*ay-c23_2*az;
theta4_3=atan2(p1_2,g1_2);
theta4_4=atan2(-p1_2,-g1_2);

%theta5
theta5_1=atan2(cos(theta4_1)*g1_1+sin(theta4_1)*p1_1,-sin(theta1_1)*ax+cos(theta1_1)*ay);
theta5_2=atan2(cos(theta4_2)*g1_1+sin(theta4_2)*p1_1,-sin(theta1_1)*ax+cos(theta1_1)*ay);
theta5_3=atan2(cos(theta4_3)*g1_2+sin(theta4_3)*p1_2,-sin(theta1_1)*ax+cos(theta1_1)*ay);
theta5_4=atan2(cos(theta4_4)*g1_2+sin(theta4_4)*p1_2,-sin(theta1_1)*ax+cos(theta1_1)*ay);

%theta6
theta6_1=atan2(-sin(theta1_1)*ox+cos(theta1_1)*oy,sin(theta1_1)*nx-cos(theta1_1)*ny);
theta6_2=atan2(sin(theta1_1)*ox-cos(theta1_1)*oy,-(sin(theta1_1)*nx-cos(theta1_1)*ny));
theta6_3=atan2(-sin(theta1_1)*ox+cos(theta1_1)*oy,sin(theta1_1)*nx-cos(theta1_1)*ny);
theta6_4=atan2(sin(theta1_1)*ox-cos(theta1_1)*oy,-(sin(theta1_1)*nx-cos(theta1_1)*ny));

% Turn around , because the six joint is 360 degrees free
if theta6_4 == pi
    theta6_4 = -pi;
end
     
theta=[theta1_1 theta2_1 theta3_1 theta4_1 theta5_1 theta6_1
       theta1_1 theta2_1 theta3_1 theta4_2 theta5_2 theta6_2
       theta1_1 theta2_2 theta3_2 theta4_3 theta5_3 theta6_3
       theta1_1 theta2_2 theta3_2 theta4_4 theta5_4 theta6_4]*180/pi;


for i = 1:4
    if theta(i,1) < -150 || theta(i,1) > 150
       continue
    end    
    if theta(i,2) < -30 || theta(i,2) > 100
       continue
    end 
    if theta(i,3) < -120 || theta(i,3) > 0
       continue
    end 
    if theta(i,4) < -110 || theta(i,4) > 110
       continue
    end 
    if theta(i,5) < -180 || theta(i,5) > 180
       continue
    end 
    if theta(i,6) < -180 || theta(i,6) > 180
       continue
    end 
end

% We choose the forth solution
theta_correct = theta(4,:);