// PID controller Altitud

function [delta,Error]=PIDcontroller(t,EstState,delta)
  global CtrlIntg1 
  global CtrlIntg2

  P_D=EstState(3); //Posicion Down en F^i
  w=EstState(6); // Velocidad z en F^b (añadido)
  phi=EstState(7); // Roll angle defined with respect to F^v2
  theta=EstState(8); // Pitch angle defined with respect to F^v1
  psi=EstState(9); // Heading (yaw) angle defined with respect to F^v
  VelAng=EstState(10:12); // [p,q,r] 

  Sphi=sin(phi); Cphi=cos(phi);
  Stheta=sin(theta); Ctheta=cos(theta);
  Spsi=sin(psi); Cpsi=cos(psi);
  Ttheta=tan(theta);

  R33=[1,Sphi*Ttheta,Cphi*Ttheta;
  0,Cphi,-Sphi;
  0,Sphi/Ctheta,Cphi/Ctheta];

  Rvtov1=[Cpsi,Spsi,0;-Spsi,Cpsi,0;0,0,1];
  Rv1tov2=[Ctheta,0,-Stheta;0,1,0;Stheta,0,Ctheta];
  Rv2tob=[1,0,0;0,Cphi,Sphi;0,-Sphi,Cphi];
  Rvtob=Rv2tob*Rv1tov2*Rvtov1;

//  dotAng=R33*VelAng; 
  dotAng=VelAng; 
  dotPos=Rvtob'*EstState(4:6);

  //Error para z
  e_pz=PD_deseado-P_D;
  e_dz=0-dotPos(3);
  printf("h_deseado=%f h=%f err_p=%f \n",PD_deseado,-P_D,e_pz);
  printf("VelocidadZ=%f  err_d=%f \n",-dotPos(3),e_dz);

  //Error para phi  
  e_pPhi=phi_deseado-phi;
  e_dPhi=0-dotAng(1);
  printf("phi_deseado=%f phi=%f err_pPhi=%f \n",phi_deseado,phi,e_pPhi);
  printf("VelocidadPhi=%f  err_dPhi=%f \n",dotAng(1),e_dPhi);

  //Error para theta
  e_pTheta=theta_deseado-theta;
  e_dTheta=0-dotAng(2);
  printf("theta_deseado=%f theta=%f err_pTheta=%f \n",theta_deseado,theta,e_pTheta);
  printf("VelocidadTheta=%f  err_dTtheta=%f \n",dotAng(2),e_dTheta);

  //Error para psi
  e_pPsi=psi_deseado-psi;
  e_dPsi=0-dotAng(3);
  printf("psi_deseado=%f psi=%f err_pPsi=%f \n",psi_deseado,psi,e_pPsi);
  printf("VelocidadPsi=%f  err_dPsi=%f \n",dotAng(3),e_dPsi);
  
  //PD controller output
  u_z=KpZ*e_pz+KdZ*e_dz;
  u_phi=KpPhi*e_pPhi+KdPhi*e_dPhi;
  u_theta=KpTheta*e_pTheta+KdTheta*e_dTheta;
  u_psi=KpPsi*e_pPsi+KdPsi*e_dPsi;
  
  Error=[e_pPhi,e_dPhi,e_pTheta,e_dTheta,e_pPsi,e_pPsi,e_pz,e_dz];

  delta(1:4)=P.m*(P.g-u_z)/(4*P.k_1*Cphi*Ctheta);

  delta(1)=delta(1)+P.J_y*u_theta/(2*P.L*P.k_1)-P.J_z*u_psi/4/P.k_2;
  delta(2)=delta(2)-P.J_x*u_phi/(2*P.L*P.k_1)+P.J_z*u_psi/4/P.k_2;
  delta(3)=delta(3)-P.J_y*u_theta/(2*P.L*P.k_1)-P.J_z*u_psi/4/P.k_2;
  delta(4)=delta(4)+P.J_x*u_phi/(2*P.L*P.k_1)+P.J_z*u_psi/4/P.k_2;

  printf("d1=%f,d2=%f,d3=%f,d4=%f\n",delta(1),delta(2),delta(3),delta(4))
endfunction
