% Equation of motions *****************************
function dQ = EOMFlight(t,Q,param)
    % Describe the system equation of motion
    % Reminder: the state Q = [q dq]; state output dQ = [dq ddq]
    dQ = zeros(2,1);
    dQ(1) = Q(2);
    dQ(2) = -param.g;
end