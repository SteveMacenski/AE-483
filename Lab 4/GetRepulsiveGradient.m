function gradfrep = GetRepulsiveGradient(drone,obst,param)
gradfrep = [0 0 0];
for i=1:length(obst)
    
    if (obst{i}.type == 1)
        [d,dgrad] = SphereSphere(drone.q,drone.r,obst{i}.p,obst{i}.s);
    elseif (obst{i}.type == 2)
        [d,dgrad] = SphereHotDog(drone.q,drone.r,obst{i}.p1,obst{i}.p2,obst{i}.s);
    elseif (obst{i}.type == 3)
        [d,dgrad] = SpherePlane(drone.q,drone.r,obst{i}.p,obst{i}.z);
    else
        error('bad obst{i}.type in GetRepulsiveGradient');
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % YOUR CODE HERE (PROBLEM 5)
    %
    % param.krep and param.brep are the relevant constants
    %
      if d<=param.brep
        gradfrep_i = -( param.krep*((1/d)-(1/param.brep)).*(1/d.^2)*dgrad);
      else
        gradfrep_i = [0 0 0];
      end
      gradfrep=gradfrep+gradfrep_i;
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end
