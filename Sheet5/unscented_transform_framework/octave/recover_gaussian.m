function [mu, sigma] = recover_gaussian(sigma_points, w_m, w_c)
% This function computes the recovered Gaussian distribution (mu and sigma)
% given the sigma points (size: nx2n+1) and their weights w_m and w_c:
% w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n].
% The weight vectors are each 1x2n+1 in size,
% where n is the dimensionality of the distribution.

% Try to vectorize your operations as much as possible

% TODO: compute mu

mu = sum(w_m.*sigma_points,2);

% TODO: compute sigma

% 1_ Compute the quantity of (x-u) for the covarience matrix calculation,
%in matrix form

sigma_points_shifted = sigma_points - mu;

%2_ Compute the first item (w_c(i)*(x-mu)) in Sigma = Sum(w_c(i)(x-u)*(x-u)')
% in matrix form
weighted_sigma_points_shifted = w_c.*sigma_points_shifted;

%3_ Finally, we perform the outer product of the two matrices 

sigma = weighted_sigma_points_shifted * sigma_points_shifted'; 

end
