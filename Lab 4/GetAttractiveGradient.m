function gradfatt = GetAttractiveGradient(drone,goal,param)
if norm(drone.q-goal.q)<=param.batt
    gradfatt = param.katt*(drone.q-goal.q)';
else
    gradfatt = param.katt*param.batt*((drone.q-goal.q)/norm(drone.q-goal.q))';
end