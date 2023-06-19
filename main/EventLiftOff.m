% Event function *********************
function [value,isterminal,direction] = EventLiftOff(t, Q, param)
    % Locate the time when height passes through zero in a decreasing direction
    % and stop integration.
    value = [Q(1)-param.r];     % event point: lift off
    direction = [1];            % direction
    isterminal = [1];           % stop the integration
end