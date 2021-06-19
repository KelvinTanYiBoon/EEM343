%This is the part that derived the Transformation matrix of the robot arm
syms L1 L2 L3 L4 L5 L6 L7 th1 th2 th3 th4 th5 th6 th7 D1 D2 D3 D4 D5 D6 D7
alphaa=[0,90,0,0,-90,90,0]; % this is the alpha value for links
a=[0,0,0.24365,0.21325,0,0,0]; % Length of the Link 
d=[0.15190, 0, 0.11985,-0.093,0.08325,0.083,0.19413]; %Offset
th=[th1,th2,th3,th4,th5,th6,th7]; % joint angle variation
%%Transformation Matrices
for i=1:7
    switch i
        case 1
            T01= [cos(th(1,i)),-sin(th(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(th(1,i)),a(1,i)*cos(th(1,i));sin(th(1,i)),cos(th(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(th(1,i)),sin(th(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 2
            T12= [cos(th(1,i)),-sin(th(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(th(1,i)),a(1,i)*cos(th(1,i));sin(th(1,i)),cos(th(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(th(1,i)),sin(th(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 3
            T23= [cos(th(1,i)),-sin(th(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(th(1,i)),a(1,i)*cos(th(1,i));sin(th(1,i)),cos(th(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(th(1,i)),sin(th(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 4
            T34= [cos(th(1,i)),-sin(th(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(th(1,i)),a(1,i)*cos(th(1,i));sin(th(1,i)),cos(th(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(th(1,i)),sin(th(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 5
            T45= [cos(th(1,i)),-sin(th(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(th(1,i)),a(1,i)*cos(th(1,i));sin(th(1,i)),cos(th(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(th(1,i)),sin(th(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 6
            T56= [cos(th(1,i)),-sin(th(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(th(1,i)),a(1,i)*cos(th(1,i));sin(th(1,i)),cos(th(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(th(1,i)),sin(th(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 7
            T6E= [cos(th(1,i)),-sin(th(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(th(1,i)),a(1,i)*cos(th(1,i));sin(th(1,i)),cos(th(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(th(1,i)),sin(th(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
 
    end
end
 
T0E = T01*T12*T23*T34*T45*T56*T6E;
T0E = simplify(T0E);
A_matrix=T0E;
% J = jacobian([cos(th1 + th2)*(D3 + D4 + D5) – D6*sin(th1 + th2)*sin(th3 + th4 + th5) + D7*sin(th6)*[(-sin(th1 + th2)*cos(th3 + th4 + th5)) + cos(th1 + th2)*cos(th6)] – a4*sin(th1 + th2)*cos(th3 + th4) – a3*sin(th1 + th2)*cos(th3)], [cos(th1 + th2)*(D3 + D4 + D5) – D6*sin(th1 + th2)*sin(th3 + th4 + th5) + D7*sin(th6)*sin(th3 + th4 + th5) + D7*sin(th6)*[-sin(th1 + th2)*cos(th3 + th4 + th5) + cos(th1 + th2)*cos(th6)] – a4*sin(th1 + th2)*cos(th3 + th4) – a3*sin(th1 + th2)* cos(th3)], [D6*cos(th1 + th2)*cos(th3 + th4 + th5) – D7*cos(th1 + th2)*sin(th3 + th4 + th5)*sin(th6) – a4*cos(th1 + th2)*sin(th3 + th4) – a3*cos(th1 + th2)*sin(th3)], [D6*cos(th1 + th2)*cos(th3 + th4 + th5) – D7*cos(th1 + th2)*sin(th3 + th4 + th5)*sin(th6) – a4*cos(th1 + th2)*sin(th3 + th4)], [D6*cos(th1 + th2)*cos(th3 + th4 + th5) – D7*cos(th1 + th2)*sin(th3 + th4 + th5)*sin(th6)], [D7*sin(th1 + th2)*(cos(th6)*cos(th6) – sin(th6)*sin(th6)) + D7*cos(th6)*cos(th1 + th2)*sin(th3 + th4 + th5)], [sin(th1 + th2)*(D3 + D4 + D5) – D6*cos(th1 + th2)*sin(th3 + th4 + th5) + D7*(sin(th1 + th2)*cos(th6) + cos(th1 + th2)*cos(th3 + th4 + th5)*sin(th6)) + a4*cos(th1 + th2)*cos(th3 + th4) + a3*cos(th1 + th2)*cos(th3)], [sin(th1 + th2)*(D3 + D4 + D5) – D6*cos(th1 + th2)*sin(th3 + th4 + th5) + D7*[sin(th1 + th2)*cos(th6) + cos(th1 + th2)*cos(th3 + th4 + th5)*sin(th6)] + a4*cos(th1 + th2)*cos(th3 + th4) + a3*cos(th1 + th2)*cos(th3)], [-D6*sin(th1 + th2)*cos(th3 + th4 + th5) – D7*sin(th1 + th2)*sin(th3 + th4 + th5)*sin(th6) – a4*sin(th1 + th2)*sin(th3 + th4) – a3*sin(th1 + th2)*sin(th3)], [-D6*sin(th1 + th2)*cos(th3 + th4 + th5) – D7*sin(th1 + th2)*sin(th3 + th4 + th5)*sin(th6) – a4*sin(th1 + th2)*sin(th3 + th4)], [-D6*sin(th1 + th2)*cos(th3 + th4 + th5) – D7*sin(th1 + th2)*sin(th3 + th4 + th5)*sin(th6)], [D7*(cos(th1 + th2)*sin(th6) + sin(th1 + th2)*cos(th3 + th4 + th5)*cos(th6))], 0, 0, [-D6*sin(th3 + th4 + th5) + D7*cos(th3 + th4 + th5)*sin(th6) + a4*cos(th3 + th4) + a3*cos(th3)], [-D6*sin(th3 + th4 + th5) + D7*cos(th3 + th4 + th5)*sin(th6) + a4*cos(th3 + th4)], [-D6*sin(th3 + th4 + th5) + D7*cos(th3 + th4 + th5)*sin(th6)], [D7*sin(th3 + th4 + th5)*cos(th6)], [th1; th2; th3; th4; th5; th6])
 
%%
%This is the part for stating all links' specification and also creating 02,03&04 T matrices 
 
syms th1 th2 th3 th4 th5 th6 d1 d2 d3 d4 d5 d6 d7;
d1=0.15190; d2=0; d3=0.11985; d4=-0.093; d5=0.08325; d6=0.083; d7=0.19413; %arm architecture (m)
th_matrix=[th1 th2 th3 th4 th5 th6];
 
T02=T01*T12;
T03=T02*T23;
T04=T03*T34;
T05=T04*T45;
T06=T05*T56;
T07=T06*T6E;
 
%links specs
%mass in kg of each link
m1=4.02; m2=1.399; m3=2.5; m4=2.5; m5=1; m6=1; m7=0.09984;
 
%Position of the center of mass in the DH framework, units in m
r1=[0; 0; 36.25e-3; 0]; 
r2=[10.91e-03; 0; 6.22e-3; 0];
r3=[131.9e-3; 0; 5.21e-3; 0];
r4=[106.25e-3; 0; 5.43e-3; 0];
r5=[0; 3.92e-3; -41.09e-3; 0];
r6=[4.58e-3; 0; 8.65e-3; 0];
r7=[-63.07e-3; 0; 0; 0];
   
%%
%This calculate the Uij and Uijk matrices 
 
   %Uij Matrixes
   U11=diff(T01,th1);    U21=diff(T02,th1);    U31=diff(T03,th1);
   U12=zeros(6);         U22=diff(T02,th2);    U32=diff(T03,th2);
   U13=zeros(6);         U23=zeros(6);         U33=diff(T03,th3);
   U14=zeros(6);         U24=zeros(6);         U34=zeros(6);  
   U15=zeros(6);         U25=zeros(6);         U35=zeros(6);       
   U16=zeros(6);         U26=zeros(6);         U36=zeros(6);       
 
   
   U41=diff(T04,th1);    U51=diff(T05,th1);    U61=diff(T06,th1);
   U42=diff(T04,th2);    U52=diff(T05,th2);    U62=diff(T06,th2);
   U43=diff(T04,th3);    U53=diff(T05,th3);    U63=diff(T06,th3);
   U44=diff(T04,th4);    U54=diff(T05,th4);    U64=diff(T06,th4);
   U45=zeros(6);         U55=diff(T05,th5);    U65=diff(T06,th5);
   U46=zeros(6);         U56=zeros(6);         U66=diff(T06,th6);
 
   %Uijk Matrixes
   
   %U1jk Matrixes
   U111=diff(U11,th1); U121=zeros(6)     ; U131=zeros(6);
   U112=zeros(6)     ; U122=zeros(6)     ; U132=zeros(6);
   U113=zeros(6)     ; U123=zeros(6)     ; U133=zeros(6);
   U114=zeros(6)     ; U124=zeros(6)     ; U134=zeros(6);
   U115=zeros(6)     ; U125=zeros(6)     ; U135=zeros(6);
   U116=zeros(6)     ; U126=zeros(6)     ; U136=zeros(6);
 
   
   U141=zeros(6)     ; U151=zeros(6)     ; U161=zeros(6);
   U142=zeros(6)     ; U152=zeros(6)     ; U162=zeros(6);
   U143=zeros(6)     ; U153=zeros(6)     ; U163=zeros(6);
   U144=zeros(6)     ; U154=zeros(6)     ; U164=zeros(6);
   U145=zeros(6)     ; U155=zeros(6)     ; U165=zeros(6);
   U146=zeros(6)     ; U156=zeros(6)     ; U166=zeros(6);
   
   %U2jk Matrixes
   U211=diff(U21,th1); U221=diff(U22,th1); U231=zeros(6);
   U212=diff(U21,th2); U222=diff(U22,th2); U232=zeros(6);
   U213=zeros(6)     ; U223=zeros(6)     ; U233=zeros(6);
   U214=zeros(6)     ; U224=zeros(6)     ; U234=zeros(6);
   U215=zeros(6)     ; U225=zeros(6)     ; U235=zeros(6);
   U216=zeros(6)     ; U226=zeros(6)     ; U236=zeros(6);
 
   
   U241=zeros(6)     ; U251=zeros(6)     ; U261=zeros(6);
   U242=zeros(6)     ; U252=zeros(6)     ; U262=zeros(6);
   U243=zeros(6)     ; U253=zeros(6)     ; U263=zeros(6);
   U244=zeros(6)     ; U254=zeros(6)     ; U264=zeros(6);
   U245=zeros(6)     ; U255=zeros(6)     ; U265=zeros(6);
   U246=zeros(6)     ; U256=zeros(6)     ; U266=zeros(6);
   
   %U3jk Matrixes
   U311=diff(U31,th1); U321=diff(U32,th1); U331=diff(U33,th1);
   U312=diff(U31,th2); U322=diff(U32,th2); U332=diff(U33,th2);
   U313=diff(U31,th3); U323=diff(U32,th3); U333=diff(U33,th3);
   U314=zeros(6)     ; U324=zeros(6)     ; U334=zeros(6);
   U315=zeros(6)     ; U325=zeros(6)     ; U335=zeros(6);
   U316=zeros(6)     ; U326=zeros(6)     ; U336=zeros(6);
 
   
   U341=zeros(6)     ; U351=zeros(6)     ; U361=zeros(6);
   U342=zeros(6)     ; U352=zeros(6)     ; U362=zeros(6);
   U343=zeros(6)     ; U353=zeros(6)     ; U363=zeros(6);
   U344=zeros(6)     ; U354=zeros(6)     ; U364=zeros(6);
   U345=zeros(6)     ; U355=zeros(6)     ; U365=zeros(6);
   U346=zeros(6)     ; U356=zeros(6)     ; U366=zeros(6);
   
   %U4jk Matrixes
   U411=diff(U41,th1); U421=diff(U42,th1); U431=diff(U43,th1);
   U412=diff(U41,th2); U422=diff(U42,th2); U432=diff(U43,th2);
   U413=diff(U41,th3); U423=diff(U42,th3); U433=diff(U43,th3);
   U414=diff(U41,th4); U424=diff(U42,th4); U434=diff(U43,th4);
   U415=zeros(6)     ; U425=zeros(6)     ; U435=zeros(6);
   U416=zeros(6)     ; U426=zeros(6)     ; U436=zeros(6);
 
   
   U441=diff(U44,th1); U451=zeros(6)     ; U461=zeros(6);
   U442=diff(U44,th2); U452=zeros(6)     ; U462=zeros(6);
   U443=diff(U44,th3); U453=zeros(6)     ; U463=zeros(6);
   U444=diff(U44,th4); U454=zeros(6)     ; U464=zeros(6);
   U445=zeros(6)     ; U455=zeros(6)     ; U465=zeros(6);
   U446=zeros(6)     ; U456=zeros(6)     ; U466=zeros(6);
   
   %U5jk Matrixes
   U511=diff(U51,th1); U521=diff(U52,th1); U531=diff(U53,th1);
   U512=diff(U51,th2); U522=diff(U52,th2); U532=diff(U53,th2);
   U513=diff(U51,th3); U523=diff(U52,th3); U533=diff(U53,th3);
   U514=diff(U51,th4); U524=diff(U52,th4); U534=diff(U53,th4);
   U515=diff(U51,th5); U525=diff(U52,th5); U535=diff(U53,th5);
   U516=zeros(6)     ; U526=zeros(6)     ; U536=zeros(6);
 
   
   U541=diff(U54,th1); U551=diff(U55,th1); U561=zeros(6);
   U542=diff(U54,th2); U552=diff(U55,th2); U562=zeros(6);
   U543=diff(U54,th3); U553=diff(U55,th3); U563=zeros(6);
   U544=diff(U54,th4); U554=diff(U55,th4); U564=zeros(6);
   U545=diff(U54,th5); U555=diff(U55,th5); U565=zeros(6);
   U546=zeros(6)     ; U556=zeros(6)     ; U566=zeros(6);
 
   %U6jk Matrixes
   U611=diff(U61,th1); U621=diff(U62,th1); U631=diff(U63,th1);
   U612=diff(U61,th2); U622=diff(U62,th2); U632=diff(U63,th2);
   U613=diff(U61,th3); U623=diff(U62,th3); U633=diff(U63,th3);
   U614=diff(U61,th4); U624=diff(U62,th4); U634=diff(U63,th4);
   U615=diff(U61,th5); U625=diff(U62,th5); U635=diff(U63,th5);
   U616=diff(U61,th6); U626=diff(U62,th6); U636=diff(U63,th6);
 
   
   U641=diff(U64,th1); U651=diff(U65,th1); U661=diff(U66,th1);
   U642=diff(U64,th2); U652=diff(U65,th2); U662=diff(U66,th2);
   U643=diff(U64,th3); U653=diff(U65,th3); U663=diff(U66,th3);
   U644=diff(U64,th4); U654=diff(U65,th4); U664=diff(U66,th4);
   U645=diff(U64,th5); U655=diff(U65,th5); U665=diff(U66,th5);
   U646=diff(U64,th6); U656=diff(U65,th6); U666=diff(U66,th6);
   %%
   %Stating I and calculate inertial J 
   %I matrixes
   
  I1=[10.2921e-3     -2.36e-9       -14.35e-9;
      -2.36e-9       10.2921e-3     -11.91e-9;
      -14.35e-9      -11.91e-9      5.3716e-3];
  
  I2=[1.3791e-3      0.34e-9        -5.0677e-5;
      0.34e-9        2.2137e-3      0.49e-9;
      -5.0677e-5     0.49e-9        2.0081e-3];
 
  I3=[26.8253e-3     -0.01e-9       3.0074e-3;
      -0.01e-9       70.329e-3      2.62e-9;
      3.0074e-3      2.62e-9        69.5364e-3];
 
  I4=[2.0312e-3      -11.63e-9      2.3815e-3;
      -11.63e-9      47.4942e-3     0.07e-9;
      0.07e-9        47.0847e-3     47.0847e-3];
 
  I5=[2.5784e-3      0.29e-9        0.47e-9;
      0.29e-9        2.5278e-3      -2.0021e-4;
      -2.0021e-4     6.9373e-4      6.9373e-4];
 
  I6=[9.9189e-4      0.71e-9        -2.2094e-5;
      0.71e-9        1.0778e-3      -0.36e-9;
      -0.36e-9       6.3909e-4      6.3909e-4];
 
  I7=[5.0501e-5      0              0;
      0              4.3024e-4      0;
      0              4.3024e-4      4.3024e-4];
 
  %J matrix
  
  I=zeros(6);
  for i=1:7
      I=eval(['I' num2str(i)]);
      m=eval(['m' num2str(i)]);
      r=eval(['r' num2str(i)]);
      eval(['j11' '=((-I(1,1)+I(2,2)+I(3,3))/2)']);
      eval(['j12' '=I(1,2)']);
      eval(['j13' '=I(1,3)']);
      eval(['j14' '=m*r(1)']);
      
      eval(['j21' '=I(1,2)']);
      eval(['j22' '=((I(1,1)-I(2,2)+I(3,3))/2)']);
      eval(['j23' '=I(2,3)']);
      eval(['j24' '=m*r(2)']);
      
      eval(['j31' '=I(1,3)']);
      eval(['j32' '=I(2,3)']);
      eval(['j33' '=((I(1,1)+I(2,2)-I(3,3))/2)']);
      eval(['j34' '=m*r(3)']);
      
      eval(['j41' '=m*r(1)']);
      eval(['j42' '=m*r(2)']);
      eval(['j43' '=m*r(3)']);
      eval(['j44' '=m']);
 
      J=[j11 j12 j13 j14; j21 j22 j23 j24; j31 j32 j33 j34; j41 j42 j43 j44];
      eval(['J' num2str(i) '=J']);
  end
  
  %%
  %Finding all the D terms from U and J before and create D matrix (angular acceleration-inertia term)
  
  %D matrix
  Uaux=zeros(6);
for i=1:6
   for j=1:6
       m=max([i j]);
       x=1;
       for k=m:6
       Uaux=eval(['U' num2str(k) num2str(i)]);
       Ud=Uaux';
       A=eval(['U' num2str(k) num2str(j)])*eval(['J' num2str(k)])*Ud;
       x=x+trace(A);
       end
       eval(['d' num2str(i) num2str(j) '=x']);
   end
end
 
D=[d11 d12 d13 d14 d15 d16
   d21 d22 d23 d24 d25 d26
   d31 d32 d33 d34 d35 d36
   d41 d42 d43 d44 d45 d46
   d51 d52 d53 d54 d55 d56
   d61 d62 d63 d64 d65 d66];
 
 %actuator inertia term and Coriolis are calculated as H matrix
 
  for i=1:6
   for k=1:6
     for m=1:6
       j=max([i m k]);
       x=0;
       for l=j:6
       Uaux=eval(['U' num2str(j) num2str(i)]);
       Uh=Uaux';
       x=x+trace(eval(['U' num2str(j) num2str(k) num2str(m)])*eval(['J' num2str(j)])*Uh);
       end
       eval(['h' num2str(i) num2str(k) num2str(m) '=x']);
     end 
   end
  end
  
  
  %Coriolis column matrix
 syms dth1 dth2 dth3 dth4 dth5 dth6 
 
  for i=1:6
      y=0;
      for k=1:6 
          for m=1:6
          y=y+eval(['h' num2str(i) num2str(k) num2str(m)])*eval(['dth' num2str(k)])*eval(['dth' num2str(m)]);
          end    
      end
      eval(['h' num2str(i) '=y']);
  end
  
  H=[h1;h2;h3;h4;h5;h6];
  
  %Gravity column matrix as C
  
  g=[0 0 -9.81 0];
  
  for i=1:6
      x=0;
      for j=i:6
      x=x+(-eval(['m' num2str(j)])*g*eval(['U' num2str(j) num2str(i)])*eval(['r' num2str(j)]));  
      end
      eval(['c' num2str(i) '=x']);
  end
   
  C=[c1;c2;c3;c4;c5;c6];
  
  %%
  %Final dynamic model, T with following terms:
  % -1st part is angular acceleration-inertia term D*ddth
  % -2nd part is Coriolis and Centrifugal term H
  % -3rd part is gravity terms C
  
  syms ddth1 ddth2 ddth3 ddth4 ddth5 ddth6 
  ddth=[ddth1;ddth2;ddth3;ddth4;ddth5;ddth6]; 
  %T=[t1;t2;t3;t4;t5;t6;t7]; %Matrix with torques of each joint
  
  T=vpa(D*ddth+H+C);
  %%T = simplify(T);
