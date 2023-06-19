% Equation of motions *****************************
function dQ = EOMAirSpring(t, Q, param)
    % Describe the system equation of motion
    % Reminder: the state Q = [q dq]; state output dQ = [dq ddq]
    dQ = zeros(2,1);
    dQ(1) = Q(2);
    dQ(2) = param.k/param.m*(1/Q(1))-param.b/param.m*Q(2)-param.g;
end