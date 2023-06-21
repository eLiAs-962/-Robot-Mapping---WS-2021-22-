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
    Xi      = v2t(xi);
    Zij     = v2t(zij);
    Xj      = v2t(xj);
    tij     = Zij(1:2,end);
    %eij     = [ Rij'*(Ri'*(xj(1:2) - xi(1:2)) - tij); ... 
    %           xj(3) - xi(3) - zij(3)];   % error vector 
    %eij = t2v(pinv(Zij)*(pinv(v2t(xi)))*v2t(xj));
    
    eij = t2v( (Zij\(Xi\Xj)) ); % error vector 
    Fij     = eij' * omegij * eij; % scaler 
    
    Fx += Fij;

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    
    i = edge.fromIdx;
    l = edge.toIdx;
    xi = g.x(i:i+2);  % the robot pose
    xl = g.x(l:l+1);      % the landmark

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    
    zil     = edge.measurement;
    Zil     = v2t([zil;0]);
    omegil  = edge.information;
    Xi      = v2t(xi);
    Ri      = Xi(1:2,1:2);
    Zl      = v2t([xl;0]);
    % eil = Ri'*(xl - xi(1:2)) - zil; % error vector 
    eil = t2v( Zil\(Xi\Zl) )(1:2);
    Fil = eil' * omegil * eil;
    
    Fx += Fil;

  end

end
