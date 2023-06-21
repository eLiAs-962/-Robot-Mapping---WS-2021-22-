function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)

% Number of all landmarks
N = (size(mu,1)-3)/2;


%O = zeros(3,2*N);
%I = eye(2*N);
%G = [Gx O ; O' I];

% Motion noise

motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Compute the 3x3 Jacobian Gx of the motion model :

xGt = eye(3) + [0, 0, -u.t * sin( mu(3) + u.r1 ); ...
                0, 0, u.t * cos( mu(3) + u.r1 );
                0, 0, 0];

                

% TODO: Construct the full Jacobian G

%sigma = G * sigma * G' + R;

% The Jacobian G ( did it first to avoide modifying mu )



% Extracting the covarience matrices from sigma 

sigxx = sigma(1:3,1:3);
sigxm = sigma(1:3,4:end);
sigmm = sigma(4:end,4:end);

% Sigma_bar = Gt * Sigma *Gt' + Rt = G + R;
% where G = [xGt * sigxx * xGt' , xGt * sigxm ; (xGt * sigxm)' sigmm];

sigma = [xGt * sigxx * xGt' , xGt * sigxm ; (xGt * sigxm)' , sigmm] + R;


% TODO: Compute the predicted sigma after incorporating the motion

% The prediction of the robot next pose (only first 3 elements of mu)

Xbar = [mu(1); mu(2); mu(3)] + ... 
                            [ u.t * cos( mu(3) + u.r1); ... 
                              u.t * sin( mu(3) + u.r1); ... 
                              normalize_angle(u.r1 + u.r2)];
% Angle Normalization
Xbar(3) = normalize_angle(Xbar(3));
mu(1:3) = Xbar;
mu(4:end) = mu(4:end);


end
