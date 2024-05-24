
function F = stateTransitionJacobianFcn(state, dt)
    % Extract current state variables
    px = state(1);
    py = state(2);
    v = state(3);
    heading = state(4);
    omega = state(5);
    a = state(6);

    % Calculate partial derivatives for the Jacobian matrix
    d_px_d_px = 1;
    d_px_d_py = 0;
    d_px_d_v = dt * cos(heading);
    d_px_d_heading = -v * dt * sin(heading);
    d_px_d_omega = 0;
    d_px_d_a = 0;

    d_py_d_px = 0;
    d_py_d_py = 1;
    d_py_d_v = dt * sin(heading);
    d_py_d_heading = v * dt * cos(heading);
    d_py_d_omega = 0;
    d_py_d_a = 0;

    d_v_d_px = 0;
    d_v_d_py = 0;
    d_v_d_v = 1;
    d_v_d_heading = 0;
    d_v_d_omega = 0;
    d_v_d_a = dt;

    d_heading_d_px = 0;
    d_heading_d_py = 0;
    d_heading_d_v = 0;
    d_heading_d_heading = 1;
    d_heading_d_omega = dt;
    d_heading_d_a = 0;

    d_omega_d_px = 0;
    d_omega_d_py = 0;
    d_omega_d_v = 0;
    d_omega_d_heading = 0;
    d_omega_d_omega = 1;
    d_omega_d_a = 0;

    d_a_d_px = 0;
    d_a_d_py = 0;
    d_a_d_v = 0;
    d_a_d_heading = 0;
    d_a_d_omega = 0;
    d_a_d_a = 1;

    % Form the Jacobian matrix
    F = [d_px_d_px, d_px_d_py, d_px_d_v, d_px_d_heading, d_px_d_omega, d_px_d_a;
         d_py_d_px, d_py_d_py, d_py_d_v, d_py_d_heading, d_py_d_omega, d_py_d_a;
         d_v_d_px,  d_v_d_py,  d_v_d_v,  d_v_d_heading, d_v_d_omega,  d_v_d_a;
         d_heading_d_px, d_heading_d_py, d_heading_d_v, d_heading_d_heading, d_heading_d_omega, d_heading_d_a;
         d_omega_d_px, d_omega_d_py, d_omega_d_v, d_omega_d_heading, d_omega_d_omega, d_omega_d_a;
         d_a_d_px, d_a_d_py, d_a_d_v, d_a_d_heading, d_a_d_omega, d_a_d_a];
end
