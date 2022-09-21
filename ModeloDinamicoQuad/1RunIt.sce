// 1RunIt.sce

clc;
clear;
xdel(winsid());

debugMode=0;// if debugMode=1 will print intermediate data.
ManualControl=0;// If it is =1 then command parcer reeds command.txt file

exec("RigidBodyDynamicsAng.sce");
exec("GeometryQuad.sce");
exec("FrameParamQuad.sce");
exec("MDQuad.sce");
exec("SimParam.sce");
exec("Animation.sce");
exec("FigAndFiles.sce");
exec("PIDcontroller.sce");

// valores iniciales
pn0=0;// Inertial north position of the MAV en F^i
pe0=0;// Inertial east position of the MAV en F^i
pd0=-1;// Inertial down position (negative of altitude) of the MAV measured en F^i
u0=0;// Body frame velocity measured along i^b in F^b
v0=0; // Body frame velocity measured along j^b in F^b
w0=0; // Body frame velocity measured along k^b in F^b
phi0=10*%pi/180; // Roll angle defined with respect to F^v2
theta0=-10*%pi/180; // Pitch angle defined with respect to F^v1
psi0=-10*%pi/180; // Heading (yaw) angle defined with respect to F^v
p0=0*%pi/180; // Roll rate measured en i^b in F^b
q0=0*%pi/180; // Pitch rate measured along j^b in F^b
r0=0*%pi/180; // Yaw rate measured along k^b in F^b

x0=[pn0;pe0;pd0;u0;v0;w0;phi0;theta0;psi0;p0;q0;r0];

//Valores deseados 
PD_deseado=0//-5;
phi_deseado=0*%pi/180;
theta_deseado=0*%pi/180;
psi_deseado=0*%pi/180;

// variables de control
delta=[408750;408750;408750;408750];

// PID controller coefficients
//KdZ=5; KpZ=10;
//KdPhi=5; KpPhi=15;
//KdTheta=5; KpTheta=15;
//KdPsi=5; KpPsi=15;
KdZ=2.5; KpZ=1.5;
KdPhi=1.75; KpPhi=6;
KdTheta=1.75; KpTheta=6;
KdPsi=1.75; KpPsi=6;

tInit=0;
tFin=6; // segundos
step=0.01; // segundos
t=tInit:step:tFin;

if(ManualControl)
  header=1;
  commands =csvRead("commands.txt"," ",[],[],[],[],[], header);
  tn=commands(:,1);
  t=gsort([tn' t],"g","i");
  n=length(t);
  time(1)=t(1);
  j=1;
  for i=2:n
    if t(i) <> time(j) 
      j=j+1;
      time=[time,t(i)];
    end    
  end
  t=time;
end

tic();

// Flight simulation

MDResult=DynamicModel(x0, tInit, t, delta);

Rned2xyz=[1,0,0;0,-1,0;0,0,-1];
//Rned2xyz=[0,1,0;1,0,0;0,0,-1];
//Rned2xyz=[0,-1,0;-1,0,0;0,0,-1];
//Rned2xyz=[1,0,0;0,1,0;0,0,1];

MDResultP=Rned2xyz*MDResult(1:3,:);
MDResultA=Rned2xyz*MDResult(7:9,:); 
figure(1) 
plot(t,MDResultA(1,:).*180/%pi,t,MDResultA(2,:).*180/%pi,t,MDResultA(3,:).*180/%pi);
e=gce();
hl=captions(e.children,['phi(t)';'theta(t)';'psi(t)'],'in_upper_left');
//hl.fill_mode='on';
xgrid();
xlabel("Time (s)")
ylabel("Atitude (grad.)")
xtitle("Angles")

figure(3) 
plot(t,MDResultP(1,:),t,MDResultP(2,:),t,MDResultP(3,:));
e=gce();
hl=captions(e.children,['x(t)';'y(t)';'z(t)'],'in_upper_left');
xtitle("Position")
xlabel("Time (s)")
ylabel("Position (m)")
xgrid();

printf("Tiempo de calculo: %g\n",toc());
//	Graph(1,WTitles(1),subWtitles(1:6),[t;MDResult(1:6,:)]);
//	Graph(2,WTitles(2),subWtitles(7:12),[t;MDResult(7:12,:)]);
Save2File([t',MDResult'],DMResultFile,DMFileHeader);

// Animation
PosAng=[MDResult(1:3,:);MDResult(7:9,:)];
DrawTrajectory(PosAng,7);


