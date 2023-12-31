% this function solves the odometry calibration problem
% given a measurement matrix Z.
% We assume that the information matrix is the identity
% for each of the measurements
% Every row of the matrix contains
% z_i = [u'x, u'y, u'theta, ux, uy, ytheta]
% Z:	The measurement matrix
% X:	the calibration matrix
% returns the correction matrix X
function X = ls_calibrate_odometry(Z)
  % initial solution (the identity transformation)
  X = ones(3);
  %sigmai = eye(3);

  % TODO: initialize H and b of the linear system
  H = zeros(size(X(:),1)); b = zeros(size(X(:)));
  % TODO: loop through the measurements and update H and b
  for i = 1:size(Z,1)
    
    Ji = jacobian(i, Z);
    ei = error_function(i, X, Z);
    
    H = H + Ji' * Ji;
    b = b + Ji' * ei; 

  endfor
  % You may call the functions error_function and jacobian, see below
  % We assume that the information matrix is the identity.

  % TODO: solve and update the solution
    delx = - pinv(H) * b; % of dimension 9*1
    X = X + reshape(delx,3,3)';
end

% this function computes the error of the i^th measurement in Z
% given the calibration parameters
% i:	the number of the measurement
% X:	the actual calibration parameters
% Z:	the measurement matrix, each row contains first the scan-match result
%       and then the motion reported by odometry
% e:	the error of the ith measurement
function e = error_function(i, X, Z)
  % TODO compute the error of each measurement
  
    e = Z(i,1:3)' - X * Z(i,4:end)';
end

% derivative of the error function for the ith measurement in Z
% i:	the measurement number
% Z:	the measurement matrix
% J:	the jacobian of the ith measurement
function J = jacobian(i, Z)
  % TODO compute the Jacobian
  
    J = zeros(3,9);
    u_odometry = Z(i,4:end);
    J(1,1:3) = J(2,4:6) = J(3,7:end) = - u_odometry;

end
