% Event function *********************
function [value,isterminal,direction] = EventTouchDown(t, Q, param)
    % Locate the time when height passes through zero in a decreasing direction
    % and stop integration.
    value = [Q(1)-param.r];     % event point: touch down
    direction = [-1];           % direction
    isterminal = [1];           % stop the integration
end