function matrix = angletomatrix(angle)

s1 = sin(angle(1));
c1 = cos(angle(1));
s2 = sin(angle(2));
c2 = cos(angle(2));
s3 = sin(angle(3));
c3 = cos(angle(3));

matrix = [ c1*c2*c3-s1*s3     -c3*s1-c1*c2*s3    c1*s2;
           c1*s3+c2*c3*s1    c1*c3-c2*s1*s3    s1*s2;
             -c3*s2              s2*s3          c2];
end