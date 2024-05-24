function stateNext = CTRV_ext_StateFcn(state, dt)
    % Update equations for Constant Turn Rate Velocity (CTRV) model
    % Inputs:
    %   state: Structure containing previous state data (px, py, psi, v, omega, ev_k, eomega_k)
    %   dt: Time step
    % Outputs:
    %   state: Structure containing predicted state data

    % Previous state data
    px = state(1);
    py = state(2);
    psi = state(3);
    v =state(4);
    omega = state(5);
    ev = state(6);
    eomega = state(7);

    % Update equations for position
    px_next = px + dt * ((v + dt * ev) .* cos(psi + dt * (omega + dt * eomega)));
    py_next = py + dt * ((v + dt * ev) .* sin(psi + dt * (omega + dt * eomega)));

    % Update equations for heading angle, velocity, and turn rate
    psi_next = psi + dt * (omega + dt * eomega);
    v_next = v + dt * ev;
    omega_next = omega + dt * eomega;
        % Form the next state vector
    stateNext = [px_next; py_next; v_next; psi_next; omega_next; ev;eomega];
end