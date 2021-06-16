syms Q1 Q2 Q3 Q4 Q5 Q6 A3 A4 D1 D3 D4 D5 D6 D7

A=[0,90,0,0,90,-90,0]; 
a=[0,0,A3,A4,0,0,0];
d=[D1,0,D3,D4,D5,D6,D7]; 
Q=[Q1,Q2,Q3,Q4,Q5,Q6,0];


for i=1:7
    switch i
       case 1
            T01= [cos(Q(1,i)),-sin(Q(1,i))*cosd(A(1,i)),sind(A(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(A(1,i)),-sind(A(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(A(1,i)),cosd(A(1,i)),d(1,i);0,0,0,1];
        case 2
            T12= [cos(Q(1,i)),-sin(Q(1,i))*cosd(A(1,i)),sind(A(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(A(1,i)),-sind(A(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(A(1,i)),cosd(A(1,i)),d(1,i);0,0,0,1];
        case 3
            T23= [cos(Q(1,i)),-sin(Q(1,i))*cosd(A(1,i)),sind(A(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(A(1,i)),-sind(A(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(A(1,i)),cosd(A(1,i)),d(1,i);0,0,0,1];
        case 4
            T34= [cos(Q(1,i)),-sin(Q(1,i))*cosd(A(1,i)),sind(A(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(A(1,i)),-sind(A(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(A(1,i)),cosd(A(1,i)),d(1,i);0,0,0,1];
        case 5
            T45= [cos(Q(1,i)),-sin(Q(1,i))*cosd(A(1,i)),sind(A(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(A(1,i)),-sind(A(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(A(1,i)),cosd(A(1,i)),d(1,i);0,0,0,1];
        case 6
            T56= [cos(Q(1,i)),-sin(Q(1,i))*cosd(A(1,i)),sind(A(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(A(1,i)),-sind(A(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(A(1,i)),cosd(A(1,i)),d(1,i);0,0,0,1];
        case 7
            T6E= [cos(Q(1,i)),-sin(Q(1,i))*cosd(A(1,i)),sind(A(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(A(1,i)),-sind(A(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(A(1,i)),cosd(A(1,i)),d(1,i);0,0,0,1];
    end
end






