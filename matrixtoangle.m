function angle = matrixtoangle(matrix)



% We use ZYZ Euler angle
angle(1) = atan2(matrix(2,3), matrix(1,3));

angle(2) = atan2(cos(angle(1))*matrix(1,3)+sin(angle(1))*matrix(2,3), matrix(3,3));

angle(3) = atan2(-sin(angle(1))*matrix(1,1)+cos(angle(1))*matrix(2,1), -sin(angle(1))*matrix(1,2)+cos(angle(1))*matrix(2,2));

end