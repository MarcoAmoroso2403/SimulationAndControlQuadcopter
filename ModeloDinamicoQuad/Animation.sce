// Animation.sce

//*****************************************************************************
// Programa de evaluación y pruebas aleatorias automatizadas de
// los algoritmos de estimación de orientación para vehículos aéreos.
// Autores: Yuriy Kovalenko, Gilberto Bustamante Balderas, Pedro Huerta Durán
//*****************************************************************************

ScaleFact=10; //Trajectory length / Object length relation

Rned2xyz=[1,0,0;0,-1,0;0,0,-1];

function vertices= rotate3d(vertices,phi,theta,psi);
 C_phi=cos(phi);     S_phi=sin(phi);
 C_theta=cos(theta); S_theta=sin(theta);
 C_psi=cos(psi);     S_psi=sin(psi);

 R_roll=[1,   0,     0;...
         0,  C_phi, S_phi;...
         0, -S_phi, C_phi];

 R_pitch=[C_theta, 0, -S_theta;...
             0,    1,      0;...
          S_theta, 0, C_theta];

 R_yaw=[C_psi, S_psi, 0;...
       -S_psi, C_psi, 0;...
            0,     0,  1];

 R = R_roll*R_pitch*R_yaw;  
 // note that R above in rotational matrix from inertial frame to body frame.
 R = R';//Rotation matrix from body to inertial frame.
 vertices = (R*vertices')';   // rotate vertices
endfunction

// translate verticestices by pn, pe, pd
function vertices = translate(vertices,pn,pe,pd)
 vertices(:,1)=pn+vertices(:,1);
 vertices(:,2)=pe+vertices(:,2);
 vertices(:,3)=pd+vertices(:,3);
endfunction

function [vertices1,facelist1,ObjColors]=defineVehicleGeom()
 //the face numbering starts with 1 (not 0 given by lib3ds)
 if min(facelist1)==0
   facelist1 = facelist1 + 1;
 end  
 ObjColors=4*ones(length(facelist1(:,1)),1)
endfunction

function DrawTrajectory(PosAng,GraphWinNumb)
// u = fscanfMat(DynModResFile);
 clf(GraphWinNumb)
 scf(GraphWinNumb); 
 f = gcf(); //Get current figure and store the handle in f
 f.figure_name='Dynamic model visualization';
 f.figure_position=[100,100];
 f.axes_size=[800,600];
// f.figure_size=[800,600];
 [vertices,faces,ObjColors]=defineVehicleGeom();
 PosAngXYZ=zeros(PosAng);
 PosAngXYZ(1:3,:)=Rned2xyz*PosAng(1:3,:);
 PosAngXYZ(4:6,:)=Rned2xyz*PosAng(4:6,:);
 ObjSize=max(max(vertices,"r")-min(vertices,"r"));
 MaxTrj=max(PosAngXYZ(1:3,:),"c");
 MinTrj=min(PosAngXYZ(1:3,:),"c");
 Center=(MaxTrj+MinTrj)/2;
 TrjSize=max(MaxTrj-MinTrj);

 if TrjSize/ObjSize>ScaleFact
  vertices=vertices*TrjSize/ObjSize/ScaleFact;
  ObjSize=ScaleFact*ObjSize;
 end 

 lb=Center-(TrjSize+ObjSize)/2;
 rb=Center+(TrjSize+ObjSize)/2;

 [NRow,NCol]=size(faces)
 xvf=matrix(vertices(faces,1),NRow,NCol)'
 yvf=matrix(vertices(faces,2),NRow,NCol)'
 zvf=matrix(vertices(faces,3),NRow,NCol)'
 plot3d(xvf,yvf,list(zvf,ObjColors));
 fig=gcf();
 s=gce(); //the handle on the surface
 a=gca(); // get the handle of the current axes
 a.auto_scale = "off"
 a.rotation_angles=[77.75,87.25]; //turn the axes with giving angles
// a.rotation_angles=[70 50]; //turn the axes with giving angles
 a.data_bounds=[lb';rb'];

 title('Aircraft')
 xlabel('North')
 ylabel('West')
 zlabel('High')

 n=length(t(:))
 param3d1(PosAngXYZ(1,:),PosAngXYZ(2,:),PosAngXYZ(3,:));
 
 for i=1:n
  sleep(3);
  verticesC=rotate3d(vertices,PosAngXYZ(4,i),PosAngXYZ(5,i),PosAngXYZ(6,i));
  verticesC=translate(verticesC,PosAngXYZ(1,i),PosAngXYZ(2,i),PosAngXYZ(3,i));
  s.data.x=matrix(verticesC(faces,1),NRow,NCol)'
  s.data.y=matrix(verticesC(faces,2),NRow,NCol)'
  s.data.z=matrix(verticesC(faces,3),NRow,NCol)'
 // file_name="frame"+msprintf("%03d",i)+".gif";
 // xs2gif(fig, file_name);
 end
endfunction
