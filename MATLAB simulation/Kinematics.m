%%Initilisation of UR3

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
% e.g. 0t1=L(1).A(q)

% T_ee=[1 0 0 0;
%       0 1 0 0;
%       0 0 1 250.06
%       0 0 0 1];
% T_u0=[0.7071 0 -0.7071 -242.99;
%       0 -1 0 0;
%       -0.7071 0 -0.7071 0;
%       0 0 0 1];
%   
% T_u0=SE3.convert(T_u0);
% T_ee=SE3.convert(T_ee);
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
    xx(i) = trs(1);
    yy(i) = trs(2);
    zz(i) = trs(3);
end
subplot(1,2,2)
arm.plot(Q);
hold on
plot3(xx,yy,zz,'Color',[0 1 0],'LineWidth',2)

%Plotting characteristic
v_3 =diff(q_3)./diff(t1);
a_3 =diff(v_3)/0.15;

subplot(1,2,1);
plot(q_3,'Color',[1 0 0],'LineWidth',2);
hold on
grid
plot(v_3,'Color',[0 1 0],'LineWidth',2);
plot(a_3,'Color',[0 1 1],'LineWidth',2);
legend('displacement','velocity','acceleration');

%% Inverse Kinematic (using Peter Corke Robotic Toolbox)
T_target=[0 0 1 360
          1 0 0 300
          0 1 0 391
          0 0 0 1];
T_target=SE3.convert(T_target);
qi=arm.ikine(T_target,q_rest,'mask',[1 1 1 0 0 0]);
%defining boundary conditions:

condition_1=[q_rest(1) qi(1) 0 0 0.3 -0.3].';
condition_2=[q_rest(2) qi(2) 0 0 0.3 -0.3].';
condition_3=[q_rest(3) qi(3) 0 0 0.3 -0.3].';
condition_4=[q_rest(4) qi(4) 0 0 0.3 -0.3].';
condition_5=[q_rest(5) qi(5) 0 0 0.3 -0.3].';
condition_6=[q_rest(6) qi(6) 0 0 0.3 -0.3].';
condition_7=[q_rest(7) qi(7) 0 0 0.3 -0.3].';

tf=5;
Var_inv=inv([1 0 0 0 0 0
     1 tf tf^2 tf^3 tf^4 tf^5
     0 1 0 0 0 0
     0 1 2*tf 3*tf^2 4*tf^3 5*tf^4
     0 0 2 0 0 0
     0 0 2 6*tf 12*tf^2 20*tf^3]);
 
coefficient_1=Var_inv*condition_1;
coefficient_2=Var_inv*condition_2;
coefficient_3=Var_inv*condition_3;
coefficient_4=Var_inv*condition_4;
coefficient_5=Var_inv*condition_5;
coefficient_6=Var_inv*condition_6;
coefficient_7=Var_inv*condition_7;

c1=flip(coefficient_1);
c2=flip(coefficient_2);
c3=flip(coefficient_3);
c4=flip(coefficient_4);
c5=flip(coefficient_5);
c6=flip(coefficient_6);
c7=flip(coefficient_7);
dc_1=polyder(c1);
ddc_1=polyder(dc_1);
dc_4=polyder(c4);
ddc_4=polyder(dc_4);
t=0:0.1:5;
Qi=([(polyval(c1,t))' (polyval(c2,t))' (polyval(c3,t))' (polyval(c4,t))' (polyval(c5,t))' (polyval(c6,t))' (polyval(c7,t))']);

g=figure
subplot(1,2,1);
dis_1=polyval(c1,t);
vel_1=polyval(dc_1,t);
accel_1=polyval(ddc_1,t);
plot(t,dis_1,'Color',[1 0 1],'LineWidth',1.5);
hold on
grid on
plot(t,vel_1,'Color',[0 0 1],'LineWidth',1.5);
plot(t,accel_1,'Color',[1 0 0],'LineWidth',1.5);
title('Chacteristic of Joint 1');
legend('displacement(rad/s)','velocity(rad/s^2)','acceleration(rad/s^3)');

subplot(1,2,2);
dis_4=polyval(c4,t);
vel_4=polyval(dc_4,t);
accel_4=polyval(ddc_4,t);
plot(t,dis_4,'Color',[1 0 1],'LineWidth',1.5);
hold on
grid on
plot(t,vel_4,'Color',[0 0 1],'LineWidth',1.5);
plot(t,accel_4,'Color',[1 0 0],'LineWidth',1.5);
legend('displacement(rad/s)','velocity(rad/s^2)','acceleration(rad/s^3)');
title('Chacteristic of Joint 4');

f=figure;
Ti=fkine(arm,Qi);
for i=1:1:length(t)
T=Ti(i);
trs=transl(T);
xx(i) = trs(1);
yy(i) = trs(2);
zz(i) = trs(3);
end

arm.plot(Qi);
hold on
plot3(xx,yy,zz,'Color',[0 1 0],'LineWidth',2)
