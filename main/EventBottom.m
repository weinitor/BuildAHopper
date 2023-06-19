% Event function *********************
function [value,isterminal,direction] = EventBottom(t, Q, param)
    % Locate the time when height passes through zero in a decreasing direction
    % and stop integration.
    value = [Q(2)];             % event point: bottom point
    direction = [1];            % direction
    isterminal = [1];           % stop the integration
end