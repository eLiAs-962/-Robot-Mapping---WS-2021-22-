% performs one iteration of the Gauss-Newton algorithm
% each constraint is linearized and added to the Hessian

function dx = linearize_and_solve(g)

nnz = nnz_of_graph(g);

% allocate the sparse H and the vector b
H = spalloc(length(g.x), length(g.x), nnz);
b = zeros(length(g.x), 1);

needToAddPrior = true;

% compute the addend term to H and b for each of our constraints
disp('linearize and build system');
for eid = 1:length(g.edges)
  edge = g.edges(eid); % one edge !! 

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    
    %1_% a pointer to the beginning of a i^th node of size 3 of g.x
    i = edge.fromIdx; 
    %1_% a pointer to the beginning of a j^th node of size 3 of g.x
    j = edge.toIdx;
    xi = g.x(i:i+2);  % the first robot pose
    xj = g.x(j:j+2);      % the second robot pose

    % Computing the error and the Jacobians
    % eij the error vector
    % Aij Jacobian wrt xi
    % Bij Jacobian wrt xj
    
    % the measurment vector zij = (tij_x,tij_y, thij)^T 
    %all expressed in ith node refernce frame
    
    zij     = edge.measurement;
    omegij = edge.information; 

    [eij, Aij, Bij] = linearize_pose_pose_constraint(xi, xj, zij);


    % TODO: compute and add the term to H and b
    % Note : its a block matrices 
    
    % Calculate the system information matrix H
    H(i:i+2,i:i+2) += Aij' * omegij * Aij;
    H(j:j+2,j:j+2) += Bij' * omegij * Bij;
    H(i:i+2,j:j+2) += Aij' * omegij * Bij;
    H(j:j+2,i:i+2) = H(i:i+2,j:j+2)';
    
    % Calculate the b vector 
    b(i:i+2) += Aij' * omegij * eij;
    b(j:j+2) += Bij' * omegij * eij;


    if (needToAddPrior)
      % TODO: add the prior for one pose of this edge
      % This fixes one node to remain at its current location
      H(1:3,1:3) += eye(3);
      needToAddPrior = false;
    end

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose and the landmark in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    
    i = edge.fromIdx;
    l = edge.toIdx;
    xi = g.x(i:i+2);  % the robot pose
    xl = g.x(l:l+1);      % the landmark

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    zil =  edge.measurement;
    omegil = edge.information ;
    
    [eil, Ail, Bil] = linearize_pose_landmark_constraint(xi, xl,zil);


    % TODO: compute and add the term to H and b
    % Note : its a block matrices 
    
    % Calculate the system information matrix H
    H(i:i+2,i:i+2) += Ail' * omegil * Ail;
    H(l:l+1,l:l+1) += Bil' * omegil * Bil;
    H(i:i+2,l:l+1) += Ail' * omegil * Bil;
    H(l:l+1,i:i+2) = H(i:i+2,l:l+1)';
    
    % Calculate the b vector 
    b(i:i+2) += Ail' * omegil * eil;
    b(l:l+1) += Bil' * omegil * eil;


  end
end

disp('solving system');

% TODO: solve the linear system, whereas the solution should be stored in dx
% Remember to use the backslash operator instead of inverting H

dx = -H\b; 

end
