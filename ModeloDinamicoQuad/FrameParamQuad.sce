// Quadrotor frame parameters (from Andrew Gibiansky proyect) 

P.k_1 = 2.98e-6; // thrust coefficient
P.k_2 = 1.140e-7; // thrust coefficient  
P.kd=0; //global drag coefficient
//P.kd=0.25; //global drag coefficient
P.m=0.468;    //kg //Quadrotor's mass
P.L=0.225;   //m //distance from the center of mass of the quadcopter to the propellers

P.J_x=4.856e-3;     //kg*m^2
P.J_y=4.856e-3;      //kg*m^2
P.J_z=8.801e-3;      //kg*m^2

P.J=[P.J_x,   0  ,    0;
       0  , P.J_y,    0;
       0  ,   0  ,  P.J_z];
P.A_x=0.25;     //kg/s
P.A_y=0.25;     //kg/s
P.A_z=0.25;     //kg/s

P.A=[P.A_x,   0  ,    0;
       0  , P.A_y,    0;
       0  ,   0  ,  P.A_z];
