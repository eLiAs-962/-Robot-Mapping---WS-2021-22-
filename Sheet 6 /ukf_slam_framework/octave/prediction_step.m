function [mu, sigma, sigma_points] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model.
% mu: state vector containing robot pose and poses of landmarks obeserved so far
% Current robot pose = mu(1:3)
% Note that the landmark poses in mu are stacked in the order by which they were observed
% sigma: the covariance matrix of the system.
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% For computing lambda.
global scale;

% Compute sigma points
sigma_points = compute_sigma_points(mu, sigma);

% Dimensionality
n = length(mu);
% lambda
lambda = scale - n;

% TODO: Transform all sigma points according to the odometry command
% Remember to vectorize your operations and normalize angles
% Tip: the function normalize_angle also works on a vector (row) of angles
##sigma_points(1:3,:) +=  [u.t*cos( normalize_angle(sigma_points(3,:) + u.r1) );
##                         u.t*sin( normalize_angle(sigma_points(3,:) + u.r1) );
##                         repmat( normalize_angle(u.r1 + u.r2), 1, n*2+1) ];

sigma_points = motion_model(sigma_points,u)


% Computing the weights for recovering the mean
wm = [lambda/scale, repmat(1/(2*scale),1,2*n)];
wc = wm;

% TODO: recover mu.
% Be careful when computing the robot's orientation (sum up the sines and
% cosines and recover the 'average' angle via atan2)

theta = normalize_angle(atan2(sum(wm .* sin(sigma_points(3,:)),2),sum(wm .* cos(sigma_points(3,:)),2)));
mu = sum(wm.*sigma_points,2);%(wm*sigma_points')';
mu(3) = theta;


% TODO: Recover sigma. Again, normalize the angular difference
diff = sigma_points-mu;
diff(3,:) = normalize_angle(diff(3,:));
sigma = wc .* diff*diff';

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, 0];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Add motion noise to sigma
sigma += R;


end
