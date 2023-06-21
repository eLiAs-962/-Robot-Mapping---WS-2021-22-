function [x] = motion_model(Sigma_points,u)
  
  x = [Sigma_points] + ... 
                            [ u.t * cos( mu(3) + u.r1); ... 
                              u.t * sin( mu(3) + u.r1); ... 
                              normalize_angle(u.r1 + u.r2)];
  
  x(3) = normalize_angle(x(3));
  
  
end 