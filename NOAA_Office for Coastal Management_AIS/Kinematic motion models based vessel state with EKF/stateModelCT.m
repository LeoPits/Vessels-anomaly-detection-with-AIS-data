function stateNext = stateModelCT(state, dt)
    % Extract current state variables
    px = state(1);        % Current x position
    py = state(2);        % Current y position
    v = state(3);         % Current velocity
    heading = state(4);   % Current orientation (angle with respect to the x-axis)
    omega = state(5);     % Current angular velocity
    a = state(6);         % Current acceleration

    % Calculate the next state
    px_next = px + v * dt * cos(heading);   % Next x position using current velocity and orientation
    py_next = py + v * dt * sin(heading);   % Next y position using current velocity and orientation
    v_next = v + a * dt;                    % Next velocity by adding acceleration multiplied by time
    heading_next = heading + omega * dt;    % Next orientation by adding angular velocity multiplied by time
    omega_next = omega;                     % Assuming angular velocity remains constant
    a_next = a;                             % Assuming acceleration remains constant

    % Form the next state vector
    stateNext = [px_next; py_next; v_next; heading_next; omega_next; a_next];
end

