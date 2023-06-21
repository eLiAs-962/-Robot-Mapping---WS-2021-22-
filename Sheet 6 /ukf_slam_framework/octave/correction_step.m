function [mu, sigma, map] = correction_step(mu, sigma, z, map);
% Updates the belief, i.e., mu and sigma after observing landmarks,
% and augments the map with newly observed landmarks.
% The employed sensor model measures the range and bearing of a landmark
% mu: state vector containing robot pose and poses of landmarks obeserved so far.
% Current robot pose = mu(1:3)
% Note that the landmark poses in mu are stacked in the order by which they were observed
% sigma: the covariance matrix of the system.
% z: struct array containing the landmark observations.
% Each observation z(i) has an id z(i).id, a range z(i).range, and a bearing z(i).bearing
% The vector 'map' contains the ids of all landmarks observed so far by the robot in the order
% by which they were observed, NOT in ascending id order.

% For computing sigma
global scale;

% Number of measurements in this time step
m = size(z, 2);

% Measurement noise
Q = 0.01*eye(2);

for i = 1:m

	% If the landmark is observed for the first time:
	if (isempty(find(map == z(i).id)))
	    % Add new landmark to the map
            [mu, sigma, map] = add_landmark_to_map(mu, sigma, z(i), map, Q);
	    % The measurement has been incorporated so we quit the correction step
	    continue;
	endif

	% Compute sigma points from the predicted mean and covariance
        % This corresponds to line 6 on slide 32
	sigma_points = compute_sigma_points(mu, sigma);
        % Normalize!
	sigma_points(3,:) = normalize_angle(sigma_points(3,:));

	% Compute lambda
	n = length(mu);
	num_sig = size(sigma_points,2);
	lambda = scale - n;

% extract the current location of the landmark for each sigma point
% Use this for computing an expected measurement, i.e., applying the h function

% Data association
	landmarkIndex = find(map==(z(i).id));
  
% Extract the (X,Y)-position of the observed landmarks !,for all SigmaPoints
% to be updated since we observed them !
% 

	landmarkXs = sigma_points(2*landmarkIndex + 2, :);
	landmarkYs = sigma_points(2*landmarkIndex + 3, :);

	% TODO: Compute z_points (2x2n+1), which consists of predicted measurements from all sigma points
% The robot updates all landmarks cross_correlation during moivement
% This corresponds to line 7 on slide 32
 
% use the landmarks location in each sigm point to compute
% the expected measurment \hat(z)_t 

xy_pose_each_sigma_point = sigma_points(1:2,:);

delta = [landmarkXs ; landmarkYs] - [xy_pose_each_sigma_point];

range = sqrt(sum(delta .* delta,1));

%bearing = normalize_angle(atan2(delta(2,:),delta(1,:)) - mu(3));
bearing = normalize_angle(atan2(delta(2,:),delta(1,:)) - sigma_points(3,:));

z_points = [range;bearing];
        
        % setup the weight vector for mean and covariance
        wm = [lambda/scale, repmat(1/(2*scale), 1, 2*n)];
        wc = wm;

	% TODO: Compute zm, line 8 on slide 32
	% zm is the recovered expected measurement mean from z_points.
	% It will be a 2x1 vector [expected_range; expected_bearing].
        % For computing the expected_bearing compute a weighted average by
        % summing the sines/cosines of the angle

avg_range = sum(wm.*range);
avg_bearing = normalize_angle(atan2(sum(wm .* sin(bearing)),sum(wm .* cos(bearing))));
zm = [avg_range;avg_bearing];

	% TODO: Compute the innovation covariance matrix S (2x2), line 9 on slide 32
        % Remember to normalize the bearing after computing the difference

diff_z = z_points - zm;
diff_z(2,:) = normalize_angle(diff_z(2,:)) ;
Sin      = wc.*diff_z*(diff_z'); % innovation matrix
S = Sin + Q ; 

	% TODO: Compute Sigma_x_z, line 10 on slide 32
        % (which is equivalent to sigma times the Jacobian H transposed in EKF).
	% sigma_x_z is an nx2 matrix, where n is the current dimensionality of mu
        % Remember to normalize the bearing after computing the difference

diff_sigma = sigma_points - mu;
diff_sigma(3,:) = normalize_angle(diff_sigma(3,:));
Sxz = wc .* diff_sigma * diff_z' ;

	% TODO: Compute the Kalman gain, line 11 on slide 32

  K = Sxz * pinv(S);

	% Get the actual measurement as a vector (for computing the difference to the observation)
	z_actual = [z(i).range; z(i).bearing];

	% TODO: Update mu and sigma, line 12 + 13 on slide 32
        % normalize the relative bearing
        
innovation = z_actual - zm; 
innovation(2,:) = normalize_angle(innovation(2,:));
mu = mu + K*innovation;
sigma = sigma - (K*(S*(K')));

	% TODO: Normalize the robot heading mu(3)
mu(3) = normalize_angle(mu(3));

endfor

end
