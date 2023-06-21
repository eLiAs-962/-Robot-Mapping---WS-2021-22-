% Compute the error of a pose-landmark constraint
% x 3x1 vector (x,y,theta) of the robot pose
% l 2x1 vector (x,y) of the landmark
% z 2x1 vector (x,y) of the measurement, the position of the landmark in
%   the coordinate frame of the robot given by the vector x
%
% Output
% e 2x1 error of the constraint
% A 2x3 Jacobian wrt x
% B 2x2 Jacobian wrt l
function [eil, Ail, Bil] = linearize_pose_landmark_constraint(xi, xl, zil)

  % TODO compute the error and the Jacobians of the error
  
  Ri = v2t(xi)(1:2,1:2);
  ti = xi(1:2);
  eil = Ri'*(xl - ti) - zil;
  dxil = xl(1) - ti(1);
  dyil = xl(2) - ti(2);
  cthi = cos(xi(3));
  sthi = sin(xi(3));
  
  Ail = [-Ri' [-dxil*sthi + dyil*cthi;-dxil*cthi - dyil*sthi]];
  Bil = [Ri'];

end;
