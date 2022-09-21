//Modelo Dinamico de cuadricoptero

function [f_p,moments]=ForcesAndMoments(t,y,delta)

    force=P.k_1*delta(:)
    torque=P.k_2*delta(:)
    
    Vel=y(4:6); // velocidades en body frame
    phi=y(7); // Roll angle defined with respect to F^v2
    theta=y(8); // Pitch angle defined with respect to F^v1
    psi=y(9); // Heading (yaw) angle defined with respect to F^v 

    // Forces 
    f_p=[0; 0; -sum(force(:))];  

    //  Moments
    tau_phi=P.L*(force(4)-force(2))
    tau_theta=P.L*(force(1)-force(3))
    tau_psi=torque(2)+torque(4)-torque(1)-torque(3)
    moments=[tau_phi;tau_theta;tau_psi]
    
 if(debugMode==1)
    printf("En ForceMoments d1=%f,d2=%f,d3=%f,d4=%f\n",delta(1),delta(2),delta(3),delta(4))
    printf("In forces and moments force1=%f force2=%f force3=%f force4=%f\n",force(1),force(2),force(3),force(4))
    printf("In forces and moments torque1=%f torque2=%f torque3=%f torque4=%f\n",torque(1),torque(2),torque(3),torque(4))
    printf("In forces and moments f_p(1)=%f f_p(1)=%f f_p(1)=%f\n",f_p(1),f_p(2),f_p(3))
    printf("In forces and moments tau_phi=%f tau_theta=%f tau_psi=%f\n",tau_phi,tau_theta,tau_psi)
 end 
endfunction
