function stateNext = stateModelCV(state, dt)
    % Extract current state variables
    px = state(1);        % Current x position
    py = state(2);        % Current y position
    v = state(3);        % Current x velocity
    heading = state(4);   % Current orientation (angle with respect to the x-axis)
    omega = state(5);     % Current angular velocity
    a = state(6);         % Current acceleration

    % Calcola la prossima posizione utilizzando la velocità costante
    px_next = px +  v * dt * cos(heading);
    py_next = py +  v * dt * sin(heading);
    vx_next = v;
    heading_next = heading;
    omega_next = omega;
    a_next = a;

    % Form the next state vector
    stateNext = [px_next; py_next; vx_next; vy_next; heading_next; omega_next; a_next];
end