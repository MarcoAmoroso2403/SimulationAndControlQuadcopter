// SimParam.sce

P.g = 9.81; //m/s^2
P.Pstd=101325;// Standard pressure, kPa

P.Ts = 0.01; // autopilot sample rate

P.wind_n = 0;
P.wind_e = 0;
P.wind_d = 0;

// initial conditions
pn0    = 0;   // initial North position
pe0    = 0;  // initial East position
pd0    = 0;  // initial Down position (negative altitude)
u0     = 0;  // initial velocity along body x-axis
v0     = 0;  // initial velocity along body y-axis
w0     = 0;  // initial velocity along body z-axis
phi0   = 0;  // initial roll angle
theta0 = 0;  // initial pitch angle
psi0   = 0;  // initial yaw angle
p0     = 0;  // initial body frame roll rate
q0     = 0;  // initial body frame pitch rate
r0     = 0;  // initial body frame yaw rate
x0=[pn0; pe0; pd0; u0; v0; w0; phi0; theta0; psi0; p0; q0; r0];

delta=zeros(4,1);
//delta(1)=-1.03*%pi/180; // delta_e 
//delta(2)=0*%pi/180; // delta_a = 
//delta(3);    delta_r = 
//delta(4)=0.7;//    delta_t = 

wind=zeros(6,1);

