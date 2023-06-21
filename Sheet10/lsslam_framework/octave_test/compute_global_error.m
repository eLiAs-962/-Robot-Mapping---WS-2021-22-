% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)
    
    i = edge.fromIdx;
    j = edge.toIdx;

    xi = (g.x(i:i+2));  % the first robot pose
    xj = (g.x(j:j+2));  % the second robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    zij     = edge.measurement;
    omegij  = edge.information;
    Ri      = v2t(xi)(1:2,1:2);
    Zij     = v2t(zij);
    Rij     = Zij(1:2,1:2);
    tij     = Zij(1:2,end);
    eij     = [ Rij'*(Ri'*(xj(1:2) - xi(1:2)) - tij); ... 
                xj(3) - xi(3) - zij(3)];   % error vector 
    Fij     = eij' * omegij * eij; % scaler 
    
    Fx += Fij;

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.

  end

end
