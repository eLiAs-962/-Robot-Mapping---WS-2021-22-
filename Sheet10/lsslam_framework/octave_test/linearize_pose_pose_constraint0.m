% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [eij, Aij, Bij] = linearize_pose_pose_constraint(xi, xj, zij)

  % TODO compute the error and the Jacobians of the error
  
  % eij(1) = (Rij)^T( Ri^T (tj - ti) - tij)
  % tj  = (xj, yj, thj) // ti = (xi, yi, thi) all in Global coordinates
  % tij = (xij, yij, thij) the measurments of j seen from i expressed in i 
  % eij(2) = thj - thi - thij 
  
  % Extract the required parameters values 
  Ri   = v2t(xi)(1:2,1:2);
  Zij  = v2t(zij);
  Rij  = Zij(1:2,1:2);
  tij  = Zij(1:2,end);
  dxij = xj(1) - xi(1);
  dyij = xj(2) - xi(2);
  sthi = sin(xi(3));
  cthi = cos(xi(3));

  
  % calculating the error eij
  eij = [ Rij'*(Ri'*( xj(1:2) - xi(1:2) ) - tij); ... 
          xj(3) - xi(3) - zij(3)];

  
  % calculating Aij
  Aij         = zeros(3,3);
  aij         = [-Ri' [-dxij*sthi + dyij*cthi; -dxij*cthi - dyij*sthi]];
  Aij(1:2,:)  = Rij' * aij;
  Aij(3,:)    = [0 0 -1]; 
  
  % calculating Bij
  Bij         = zeros(3,3);;
  bij         = [Ri' [0;0]];
  Bij(1:2,:)  = Rij' * bij;
  Bij(3,:)    = [0 0 1];


end;
