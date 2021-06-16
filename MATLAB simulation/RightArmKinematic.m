%% Initilisation of UR3

L(1)=Link([0 151.90 0 pi/2]);
L(2)=Link([0 0 243.65 0]);
L(3)=Link([0 119.85 213.25 0]);
L(4)=Link([0 -93 0 -pi/2]);
L(5)=Link([0 83.25 0 pi/2]);
L(6)=Link([0 83 0 0]);

L(1).offset=pi/2;
L(2).offset=pi/2;
L(4).offset=-pi/2;

arm=SerialLink(L);
arm.name='ur3_r';
q_z = [0 0 0 0 0 0];
q_rest = [0 0 -pi/2 0 pi/4 0];

% To find transformation matrix
%e.g. 0t1=L(1).A(q)

T_ee=inv([1 0 0 0;
      0 1 0 0;
      0 0 1 250.06
      0 0 0 1]);

T_u0=inv([0.7071 0 -0.7071 -242.99;
      0 -1 0 0;
      -0.7071 0 -0.7071 0;
      0 0 0 1]);
   
T_u0=SE3.convert(T_u0);
T_ee=SE3.convert(T_ee);

T_init=arm.fkine(q_z);
T_rest=arm.fkine(q_rest);

%% Forward Kinematic
t1=0:0.15:3;
Q=jtraj(q_z,q_rest,t1);
T_rest=fkine(arm,Q);

for i = 1:1:length(t1)
    T = T_rest(i);
    trs = transl(T);
    q_3(i) = Q(i,3);
    q_5(i) = Q(i,5);
    xx(i) = trs(1);
    yy(i) = trs(2);
    zz(i) = trs(3);
end

f=figure;
arm.plot(Q);
hold on
plot3(xx,yy,zz,'Color',[0 1 0],'LineWidth',2)

%Plotting characteristic
v_3 =diff(q_3)./diff(t1);
a_3 =diff(v_3)/0.15;
v_5 =diff(q_5)./diff(t1);
a_5 =diff(v_5)/0.15;

g=figure;
subplot(1,2,1);
plot(t1,q_3,'Color',[1 0 0],'LineWidth',2);
hold on
grid
plot(0.15:0.15:3,v_3,'Color',[0 1 0],'LineWidth',2);
plot(0.3:0.15:3,a_3,'Color',[0 1 1],'LineWidth',2);
title ('Characteristic in Joint 3');
ylim([-2 1.5]);
ylabel('Displacement(rad), Velovity(rad/s), Acceleration(rad/s^2)');
xlabel('Time step');
legend('displacement','velocity','acceleration');

subplot(1,2,2);
plot(t1,q_5,'Color',[1 0 0],'LineWidth',2);
hold on
grid
plot(0.15:0.15:3,v_5,'Color',[0 1 0],'LineWidth',2);
plot(0.3:0.15:3,a_5,'Color',[0 1 1],'LineWidth',2);
title ('Characteristic in Joint 5');
ylabel('Displacement(rad), Velovity(rad/s), Acceleration(rad/s^2)');
xlabel('Time step');
legend('displacement','velocity','acceleration');

%% Inverse Kinematic and Path Planning ()

T_via1=[0 0 1 -170.6;
        0 -1 0 -296.5;
        1 0 0 -298.7;
        0 0 0 1];
T_via2=[0 -0.7071 0.7071 -623.1;
        0 -0.7071 -0.7071 -442.6;
        1 0 0 -338;
        0 0 0 1];
T_via3=[0 -0.7071 -0.7071 -623.1;
        0 0.7071 -0.7071 332.44;
        1 0 0 -288;
        0 0 0 1];
T_via4=[0 0 -1 -273;
        0 1 0 322.4;
        1 0 0 -238;
        0 0 0 1];
    
T_via1=T_u0*SE3.convert(T_via1)*T_ee;
T_via2=T_u0*SE3.convert(T_via2)*T_ee;
T_via3=T_u0*SE3.convert(T_via3)*T_ee;
T_via4=T_u0*SE3.convert(T_via4)*T_ee;

q_via1=q_rest;
q_via2=arm.ikine(T_via2,q_via1,'mask',[1 1 1 0 0 0]);
q_via3=arm.ikine(T_via3,q_via2,'mask',[1 1 1 0 0 0]);
q_via4=arm.ikine(T_via4,q_via3,'mask',[1 1 1 0 0 0]);

dq_via1=[0 0 0 0 0 0];
ddq_via1=[0 0 0 0 0 0];
dq_via4=[0 0 0 0 0 0];
ddq_via4=[0 0 0 0 0 0];

h1= 0:0.2:2;
h2= 0:0.2:4;
h3= 0:0.2:2;

M=inv([0 0 0 0 0 1 0 0 0 0 0 0 0 0;
   1 h1(end) h1(end)^2 h1(end)^3 h1(end)^4 0 0 0 0 0 0 0 0 0;
   0 0 2 0 0 0 0 0 0 0 0 0 0 0;
   0 1 0 0 0 0 0 0 0 0 0 0 0 0;
   1 0 0 0 0 0 0 0 0 0 0 0 0 0;
   0 0 0 0 0 0 0 0 0 1 0 0 0 0;
   0 0 0 0 0 1 h2(end) h2(end)^2 h2(end)^3 0 0 0 0 0;
   0 0 2 6*h1(end) 12*h1(end)^2 0 0 -2 0 0 0 0 0 0;
   0 1 2*h1(end) 3*h1(end)^2 4*h1(end)^3 0 -1 0 0 0 0 0 0 0;  
   0 0 0 0 0 0 0 0 0 0 0 2 6*h3(end) 12*h3(end)^2;
   0 0 0 0 0 0 0 0 0 0 1 2*h3(end) 3*h3(end)^2 4*h3(end)^3;
   0 0 0 0 0 0 0 0 0 1 h3(end) h3(end)^2 h3(end)^3 h3(end)^4;  
   0 0 0 0 0 0 0 2 6*h2(end) 0 0 -2 0 0;
   0 0 0 0 0 0 1 2*h2(end) 3*h2(end)^2 0 -1 0 0 0]);


V1=([q_via2(1) q_via2(1) ddq_via1(1) dq_via1(1) q_via1(1) q_via3(1) q_via3(1) 0 0 ddq_via4(1) dq_via4(1) q_via4(1) 0 0])';
C1=M*V1;
V2=([q_via2(2) q_via2(2) ddq_via1(2) dq_via1(2) q_via1(2) q_via3(2) q_via3(2) 0 0 ddq_via4(2) dq_via4(2) q_via4(2) 0 0])';
C2=M*V2;
V3=([q_via2(3) q_via2(3) ddq_via1(3) dq_via1(3) q_via1(3) q_via3(3) q_via3(3) 0 0 ddq_via4(3) dq_via4(3) q_via4(3) 0 0])';
C3=M*V3;
V4=([q_via2(4) q_via2(4) ddq_via1(4) dq_via1(4) q_via1(4) q_via3(4) q_via3(4) 0 0 ddq_via4(4) dq_via4(4) q_via4(4) 0 0])';
C4=M*V4;
V5=([q_via2(5) q_via2(5) ddq_via1(5) dq_via1(5) q_via1(5) q_via3(5) q_via3(5) 0 0 ddq_via4(5) dq_via4(5) q_via4(5) 0 0])';
C5=M*V5;
V6=([q_via2(6) q_via2(6) ddq_via1(6) dq_via1(6) q_via1(6) q_via3(6) q_via3(6) 0 0 ddq_via4(6) dq_via4(6) q_via4(6) 0 0])';
C6=M*V6;

% Joint 1
a_j1=[C1(5);C1(4);C1(3);C1(2);C1(1)]';
b_j1=[C1(9);C1(8);C1(7);C1(6)]';
c_j1=[C1(14);C1(13);C1(12);C1(11);C1(10)]';
da_j1=polyder(a_j1);
db_j1=polyder(b_j1);
dc_j1=polyder(c_j1);
dda_j1=polyder(da_j1);
ddb_j1=polyder(db_j1);
ddc_j1=polyder(dc_j1);

theta1_j1=polyval(a_j1,h1);
theta2_j1=polyval(b_j1,h2);
theta3_j1=polyval(c_j1,h3);
dtheta1_j1=polyval(da_j1,h1);
dtheta2_j1=polyval(db_j1,h2);
dtheta3_j1=polyval(dc_j1,h3);
ddtheta1_j1=polyval(dda_j1,h1);
ddtheta2_j1=polyval(ddb_j1,h2);
ddtheta3_j1=polyval(ddc_j1,h3);

% JOint 2
a_j2=[C2(5);C2(4);C2(3);C2(2);C2(1)]';
b_j2=[C2(9);C2(8);C2(7);C2(6)]';
c_j2=[C2(14);C2(13);C2(12);C2(11);C2(10)]';
da_j2=polyder(a_j2);
db_j2=polyder(b_j2);
dc_j2=polyder(c_j2);
dda_j2=polyder(da_j2);
ddb_j2=polyder(db_j2);
ddc_j2=polyder(dc_j2);

theta1_j2=polyval(a_j2,h1);
theta2_j2=polyval(b_j2,h2);
theta3_j2=polyval(c_j2,h3);
dtheta1_j2=polyval(da_j2,h1);
dtheta2_j2=polyval(db_j2,h2);
dtheta3_j2=polyval(dc_j2,h3);
ddtheta1_j2=polyval(dda_j2,h1);
ddtheta2_j2=polyval(ddb_j2,h2);
ddtheta3_j2=polyval(ddc_j2,h3);

% JOint 3
a_j3=[C3(5);C3(4);C3(3);C3(2);C3(1)]';
b_j3=[C3(9);C3(8);C3(7);C3(6)]';
c_j3=[C3(14);C3(13);C3(12);C3(11);C3(10)]';
da_j3=polyder(a_j3);
db_j3=polyder(b_j3);
dc_j3=polyder(c_j3);
dda_j3=polyder(da_j3);
ddb_j3=polyder(db_j3);
ddc_j3=polyder(dc_j3);

theta1_j3=polyval(a_j3,h1);
theta2_j3=polyval(b_j3,h2);
theta3_j3=polyval(c_j3,h3);
dtheta1_j3=polyval(da_j3,h1);
dtheta2_j3=polyval(db_j3,h2);
dtheta3_j3=polyval(dc_j3,h3);
ddtheta1_j3=polyval(dda_j3,h1);
ddtheta2_j3=polyval(ddb_j3,h2);
ddtheta3_j3=polyval(ddc_j3,h3);

% JOint 4
a_j4=[C4(5);C4(4);C4(3);C4(2);C4(1)]';
b_j4=[C4(9);C4(8);C4(7);C4(6)]';
c_j4=[C4(14);C4(13);C4(12);C4(11);C4(10)]';
da_j4=polyder(a_j4);
db_j4=polyder(b_j4);
dc_j4=polyder(c_j4);
dda_j4=polyder(da_j4);
ddb_j4=polyder(db_j4);
ddc_j4=polyder(dc_j4);

theta1_j4=polyval(a_j4,h1);
theta2_j4=polyval(b_j4,h2);
theta3_j4=polyval(c_j4,h3);
dtheta1_j4=polyval(da_j4,h1);
dtheta2_j4=polyval(db_j4,h2);
dtheta3_j4=polyval(dc_j4,h3);
ddtheta1_j4=polyval(dda_j4,h1);
ddtheta2_j4=polyval(ddb_j4,h2);
ddtheta3_j4=polyval(ddc_j4,h3);

% Joint 5
a_j5=[C5(5);C5(4);C5(3);C5(2);C5(1)]';
b_j5=[C5(9);C5(8);C5(7);C5(6)]';
c_j5=[C5(14);C5(13);C5(12);C5(11);C5(10)]';
da_j5=polyder(a_j5);
db_j5=polyder(b_j5);
dc_j5=polyder(c_j5);
dda_j5=polyder(da_j5);
ddb_j5=polyder(db_j5);
ddc_j5=polyder(dc_j5);

theta1_j5=polyval(a_j5,h1);
theta2_j5=polyval(b_j5,h2);
theta3_j5=polyval(c_j5,h3);
dtheta1_j5=polyval(da_j5,h1);
dtheta2_j5=polyval(db_j5,h2);
dtheta3_j5=polyval(dc_j5,h3);
ddtheta1_j5=polyval(dda_j5,h1);
ddtheta2_j5=polyval(ddb_j5,h2);
ddtheta3_j5=polyval(ddc_j5,h3);

% Joint 6
a_j6=[C6(5);C6(4);C6(3);C6(2);C6(1)]';
b_j6=[C6(9);C6(8);C6(7);C6(6)]';
c_j6=[C6(14);C6(13);C6(12);C6(11);C6(10)]';
da_j6=polyder(a_j6);
db_j6=polyder(b_j6);
dc_j6=polyder(c_j6);
dda_j6=polyder(da_j6);
ddb_j6=polyder(db_j6);
ddc_j6=polyder(dc_j6);

theta1_j6=polyval(a_j6,h1);
theta2_j6=polyval(b_j6,h2);
theta3_j6=polyval(c_j6,h3);
dtheta1_j6=polyval(da_j6,h1);
dtheta2_j6=polyval(db_j6,h2);
dtheta3_j6=polyval(dc_j6,h3);
ddtheta1_j6=polyval(dda_j6,h1);
ddtheta2_j6=polyval(ddb_j6,h2);
ddtheta3_j6=polyval(ddc_j6,h3);


%% Figure plot

f_joint1=figure
plot(h1,theta1_j1,'Color',[1 0 0],'LineWidth',1.2);
hold on;
grid on;
plot(2:0.2:6,theta2_j1,'Color',[1 0.2 0.5],'LineWidth',1.2);
plot(6:0.2:8,theta3_j1,'Color',[1 0.5 0.2],'LineWidth',1.2);
plot(h1,dtheta1_j1,'Color',[0 1 0],'LineWidth',1.2);
plot(2:0.2:6,dtheta2_j1,'Color',[0.2 1 0.5],'LineWidth',1.2);
plot(6:0.2:8,dtheta3_j1,'Color',[0.5 1 0.2],'LineWidth',1.2);
plot(h1,ddtheta1_j1,'Color',[0 0 1],'LineWidth',1.2);
plot(2:0.2:6,ddtheta2_j1,'Color',[0.2 0.5 1],'LineWidth',1.2);
plot(6:0.2:8,ddtheta3_j1,'Color',[0.5 0.2 1],'LineWidth',1.2);
title('Characteristic of Joint 1');
ylim([-1.5 3]);
ylabel('displacement(rad),velocity(rad/s),acceleration(rad/s^2)');
xlabel('time(s)');
legend('displacement_1','displacement_2','displacement_3','velocity_1','velocity_2','velocity_3','acceleration_1','acceleration_2','acceleration_3');

f_joint2=figure
plot(h1,theta1_j2,'Color',[1 0 0],'LineWidth',1.2);
hold on;
grid on;
plot(2:0.2:6,theta2_j2,'Color',[1 0.2 0.5],'LineWidth',1.2);
plot(6:0.2:8,theta3_j2,'Color',[1 0.5 0.2],'LineWidth',1.2);
plot(h1,dtheta1_j2,'Color',[0 1 0],'LineWidth',1.2);
plot(2:0.2:6,dtheta2_j2,'Color',[0.2 1 0.5],'LineWidth',1.2);
plot(6:0.2:8,dtheta3_j2,'Color',[0.5 1 0.2],'LineWidth',1.2);
plot(h1,ddtheta1_j2,'Color',[0 0 1],'LineWidth',1.2);
plot(2:0.2:6,ddtheta2_j2,'Color',[0.2 0.5 1],'LineWidth',1.2);
plot(6:0.2:8,ddtheta3_j2,'Color',[0.5 0.2 1],'LineWidth',1.2);
title('Characteristic of Joint 2');
ylim([-1 3]);
ylabel('displacement(rad),velocity(rad/s),acceleration(rad/s^2)');
xlabel('time(s)');
legend('displacement_1','displacement_2','displacement_3','velocity_1','velocity_2','velocity_3','acceleration_1','acceleration_2','acceleration_3');

f_joint3=figure
plot(h1,theta1_j3,'Color',[1 0 0],'LineWidth',1.2);
hold on;
grid on;
plot(2:0.2:6,theta2_j3,'Color',[1 0.2 0.5],'LineWidth',1.2);
plot(6:0.2:8,theta3_j3,'Color',[1 0.5 0.2],'LineWidth',1.2);
plot(h1,dtheta1_j3,'Color',[0 1 0],'LineWidth',1.2);
plot(2:0.2:6,dtheta2_j3,'Color',[0.2 1 0.5],'LineWidth',1.2);
plot(6:0.2:8,dtheta3_j3,'Color',[0.5 1 0.2],'LineWidth',1.2);
plot(h1,ddtheta1_j3,'Color',[0 0 1],'LineWidth',1.2);
plot(2:0.2:6,ddtheta2_j3,'Color',[0.2 0.5 1],'LineWidth',1.2);
plot(6:0.2:8,ddtheta3_j3,'Color',[0.5 0.2 1],'LineWidth',1.2);
title('Characteristic of Joint 3');
ylim([-2 3]);
ylabel('displacement(rad),velocity(rad/s),acceleration(rad/s^2)');
xlabel('time(s)');
legend('displacement_1','displacement_2','displacement_3','velocity_1','velocity_2','velocity_3','acceleration_1','acceleration_2','acceleration_3');

f_joint4=figure
plot(h1,theta1_j4,'Color',[1 0 0],'LineWidth',1.2);
hold on;
grid on;
plot(2:0.2:6,theta2_j4,'Color',[1 0.2 0.5],'LineWidth',1.2);
plot(6:0.2:8,theta3_j4,'Color',[1 0.5 0.2],'LineWidth',1.2);
plot(h1,dtheta1_j4,'Color',[0 1 0],'LineWidth',1.2);
plot(2:0.2:6,dtheta2_j4,'Color',[0.2 1 0.5],'LineWidth',1.2);
plot(6:0.2:8,dtheta3_j4,'Color',[0.5 1 0.2],'LineWidth',1.2);
plot(h1,ddtheta1_j4,'Color',[0 0 1],'LineWidth',1.2);
plot(2:0.2:6,ddtheta2_j4,'Color',[0.2 0.5 1],'LineWidth',1.2);
plot(6:0.2:8,ddtheta3_j4,'Color',[0.5 0.2 1],'LineWidth',1.2);
title('Characteristic of Joint 4');
ylim([-1.5 3]);
ylabel('displacement(rad),velocity(rad/s),acceleration(rad/s^2)');
xlabel('time(s)');
legend('displacement_1','displacement_2','displacement_3','velocity_1','velocity_2','velocity_3','acceleration_1','acceleration_2','acceleration_3');

f_joint5=figure
plot(h1,theta1_j5,'Color',[1 0 0],'LineWidth',1.2);
hold on;
grid on;
plot(2:0.2:6,theta2_j5,'Color',[1 0.2 0.5],'LineWidth',1.2);
plot(6:0.2:8,theta3_j5,'Color',[1 0.5 0.2],'LineWidth',1.2);
plot(h1,dtheta1_j5,'Color',[0 1 0],'LineWidth',1.2);
plot(2:0.2:6,dtheta2_j5,'Color',[0.2 1 0.5],'LineWidth',1.2);
plot(6:0.2:8,dtheta3_j5,'Color',[0.5 1 0.2],'LineWidth',1.2);
plot(h1,ddtheta1_j5,'Color',[0 0 1],'LineWidth',1.2);
plot(2:0.2:6,ddtheta2_j5,'Color',[0.2 0.5 1],'LineWidth',1.2);
plot(6:0.2:8,ddtheta3_j5,'Color',[0.5 0.2 1],'LineWidth',1.2);
title('Characteristic of Joint 5');
ylim([-3 4]);
ylabel('displacement(rad),velocity(rad/s),acceleration(rad/s^2)');
xlabel('time(s)');
legend('displacement_1','displacement_2','displacement_3','velocity_1','velocity_2','velocity_3','acceleration_1','acceleration_2','acceleration_3');

f_joint6=figure
plot(h1,theta1_j6,'Color',[1 0 0],'LineWidth',1.2);
hold on;
grid on;
plot(2:0.2:6,theta2_j6,'Color',[1 0.2 0.5],'LineWidth',1.2);
plot(6:0.2:8,theta3_j6,'Color',[1 0.5 0.2],'LineWidth',1.2);
plot(h1,dtheta1_j6,'Color',[0 1 0],'LineWidth',1.2);
plot(2:0.2:6,dtheta2_j6,'Color',[0.2 1 0.5],'LineWidth',1.2);
plot(6:0.2:8,dtheta3_j6,'Color',[0.5 1 0.2],'LineWidth',1.2);
plot(h1,ddtheta1_j6,'Color',[0 0 1],'LineWidth',1.2);
plot(2:0.2:6,ddtheta2_j6,'Color',[0.2 0.5 1],'LineWidth',1.2);
plot(6:0.2:8,ddtheta3_j6,'Color',[0.5 0.2 1],'LineWidth',1.2);
title('Characteristic of Joint 6');
ylim([-1.5 3]);
ylabel('displacement(rad),velocity(rad/s),acceleration(rad/s^2)');
xlabel('time(s)');
legend('displacement_1','displacement_2','displacement_3','velocity_1','velocity_2','velocity_3','acceleration_1','acceleration_2','acceleration_3');

%% Ploting motion

robot_plot=figure;
T_1=arm.fkine([theta1_j1' theta1_j2' theta1_j3' theta1_j4' theta1_j5' theta1_j6']);
T_2=arm.fkine([theta2_j1' theta2_j2' theta2_j3' theta2_j4' theta2_j5' theta2_j6']);
T_3=arm.fkine([theta3_j1' theta3_j2' theta3_j3' theta3_j4' theta3_j5' theta3_j6']);

for i=1:1:length(h1)
    T1=T_1(i);
    trs=transl(T1);
    xx1(i)=trs(1);
    yy1(i)=trs(2);
    zz1(i)=trs(3);
end
for i=1:1:length(h2)
    T2=T_2(i);
    trs=transl(T2);
    xx2(i)=trs(1);
    yy2(i)=trs(2);
    zz2(i)=trs(3);
end
for i=1:1:length(h3)
    T3=T_3(i);
    trs=transl(T3);
    xx3(i)=trs(1);
    yy3(i)=trs(2);
    zz3(i)=trs(3); 
end
arm.plot([theta1_j1' theta1_j2' theta1_j3' theta1_j4' theta1_j5' theta1_j6']);
hold on;
arm.plot([theta2_j1' theta2_j2' theta2_j3' theta2_j4' theta2_j5' theta2_j6']);
arm.plot([theta3_j1' theta3_j2' theta3_j3' theta3_j4' theta3_j5' theta3_j6']);
p1=plot3(xx1,yy1,zz1,'Color',[1 0 0],'LineWidth',1);
p2=plot3(xx2,yy2,zz2,'Color',[0 1 0],'LineWidth',1);
p3=plot3(xx3,yy3,zz3,'Color',[0 0 1],'LineWidth',1);
init_point=plot3(xx1(1),yy1(1),zz1(1),'.','MarkerSize',15,'Color',[0 0 0]);
plot3(xx1(end),yy1(end),zz1(end),'.','MarkerSize',15,'Color',[0 0 0]);
plot3(xx2(end),yy2(end),zz2(end),'.','MarkerSize',15,'Color',[0 0 0]);
plot3(xx3(end),yy3(end),zz3(end),'.','MarkerSize',15,'Color',[0 0 0]);
legend([p1,p2,p3,init_point],'segment 1','segment 2','segment 3','via points');
