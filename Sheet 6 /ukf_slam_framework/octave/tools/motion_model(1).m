function [Sigma_points] = motion_model(Sigma_points,u)
  
% This function updtae the system state(sigma points in this case) according to the motion model
% only the first three variables of the sigmapoints are updated, 
% that represent a candidtae pose of the robot
% The input to this function : 
%                             Sigma_points : matrix of vectors of sigma points(Nx2N+1)
%                             u : odometry motion commands (u.r1,u.t,u.r2)
% The output of this function is the transformed ( mapped) sigma points according to motion model 


% motion model : g() = (system_state)_t-1 + increment. 
% increament =  [ u.t * cos( theta_current_of_sigmapoint + u.r1); ... 
%                u.t * sin( theta_current_of_sigmapoint + u.r1); ... 
%                u.r1 + u.r2];
% increment size is 3x2N+1


increment = [ [u.t * cos( normalize_angle(Sigma_points(3,:) + u.r1))]; ... 
                              [u.t * sin( normalize_angle(Sigma_points(3,:) + u.r1) )]; ... 
                              [ones(size( Sigma_points(3,:)))*normalize_angle(u.r1 + u.r2)]];
                              

Sigma_points(1:3,:) = Sigma_points(1:3,:) + increment ; 
Sigma_points(3,:) = normalize_angle(Sigma_points(3,:));
Sigma_points = Sigma_points;
  
end 