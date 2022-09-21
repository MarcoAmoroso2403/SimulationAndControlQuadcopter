// Rigid body dynamics in 3D

function ydot = DynMod(t,y,delta)
  Vel=y(4:6); // [u,v,w]
  phi=y(7); // Roll angle defined with respect to F^v2
  theta=y(8); // Pitch angle defined with respect to F^v1
  psi=y(9); // Heading (yaw) angle defined with respect to F^v
  VelAng=y(10:12); // [p,q,r] 

  Sphi=sin(phi); Cphi=cos(phi);
  Stheta=sin(theta); Ctheta=cos(theta);
  Spsi=sin(psi); Cpsi=cos(psi);
  Ttheta=tan(theta);

  Rvtov1=[Cpsi,Spsi,0;-Spsi,Cpsi,0;0,0,1];
  Rv1tov2=[Ctheta,0,-Stheta;0,1,0;Stheta,0,Ctheta];
  Rv2tob=[1,0,0;0,Cphi,Sphi;0,-Sphi,Cphi];
  Rvtob=Rv2tob*Rv1tov2*Rvtov1;
  R33=[1,Sphi*Ttheta,Cphi*Ttheta;
  0,Cphi,-Sphi;
  0,Sphi/Ctheta,Cphi/Ctheta];

  // dynamics
  dotPos=Rvtob'*Vel;
  [f_p,moments]=ForcesAndMoments(t,y,delta) 
  f_g=Rvtob*[0;0;P.m*P.g]; //gravity
//  f_a=[0;0;0];
  f_a=-P.A*Vel
  forces=f_g+f_p+f_a;
  dotVel=forces/P.m -cross(VelAng,Vel);
  dotAng=R33*VelAng;
  dotVelAng=inv(P.J)*(moments-cross(VelAng,(P.J*VelAng)))
  ydot=[dotPos;dotVel;dotAng;dotVelAng];

  if(debugMode==1)
    printf("In DynMod d1=%f,d2=%f,d3=%f,d4=%f\n",delta(1),delta(2),delta(3),delta(4))
    printf("In DynMod force1=%f force2=%f force3=%f force4=%f\n",force(1),force(2),force(3),force(4))
    printf("In DynMod VelAng1=%f VelAng2=%f VelAng3=%f\n",VelAng(1),VelAng(2),VelAng(3))
    printf("In DynMod dotAng1=%f dotAng2=%f dotAng3=%f\n",dotAng(1),dotAng(2),dotAng(3))
    printf("In DynMod dotVelAng1=%f dotVelAng2=%f dotVelAng3=%f\n",dotVelAng(1),dotVelAng(2),dotVelAng(3))
  end 
endfunction

function y =DynamicModel(y0, tInit, t, delta)
  n=length(t);
  m=length(y0);
  y=zeros(m,n);
  y(:,1)=y0;
  Deltas=zeros(4,n);
  Deltas(:,1)=delta;

  if(ManualControl)
    m=length(commands(:,1));
  end
  for i=2:n
    dt=t(i)-t(i-1);
    printf("\ntime =%f\n",t(i));

    if(ManualControl)
      for j=1:m
        if t(i)==commands(j,1);
          delta=commands(j,2:5)';
        end
      end
    else
      [delta,Error]=PIDcontroller(t(i),y(:,i-1),delta);
    end    
    y(:,i) = ode("rk",y(:,i-1), t(i-1),t(i), list(DynMod,delta));
    if(debugMode==1)
      printf("En DynamicModel Err1=%f,Err2=%f,Err3=%f,Err4=%f\n",Error(1),Error(2),Error(3),Error(4))
      printf("En DynamicModel Err5=%f,Err6=%f,Err7=%f,Err8=%f\n",Error(5),Error(6),Error(7),Error(8))
      printf("En DynamicModel d1=%f,d2=%f,d3=%f,d4=%f\n",delta(1),delta(2),delta(3),delta(4));
      printf("En DynamicModel pn=%f,pe=%f,=%f\n",y(10,i),y(11,i),y(12,i));
      printf("En DynamicModel p=%f,q=%f,r=%f\n",y(10,i),y(11,i),y(12,i));
      printf("En DynamicModel p=%f,q=%f,r=%f\n",y(10,i),y(11,i),y(12,i));
      printf("En DynamicModel p=%f,q=%f,r=%f\n",y(10,i),y(11,i),y(12,i));
    end      
  end
endfunction
