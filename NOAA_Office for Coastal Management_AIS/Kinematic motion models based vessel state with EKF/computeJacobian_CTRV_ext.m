function F = computeJacobian_CTRV_ext(x, dt)
    % Estrai le componenti dello stato
    px = x(1);
    py = x(2);
    psi = x(3);
    v = x(4);
    omega = x(5);
    a = x(6);

    % Calcola i termini del Jacobiano
    if omega ~= 0
        F11 = 1;
        F12 = 0;
        F13 = (v/omega) * (cos(psi + omega * dt) - cos(psi));
        F14 = (1/omega) * (sin(psi + omega * dt) - sin(psi));
        F15 = (v/omega^2) * (sin(psi) - sin(psi + omega * dt)) + (v*dt/omega) * cos(psi + omega * dt);
        F16 = 0;

        F21 = 0;
        F22 = 1;
        F23 = (v/omega) * (sin(psi + omega * dt) - sin(psi));
        F24 = (1/omega) * (cos(psi) - cos(psi + omega * dt));
        F25 = (v/omega^2) * (cos(psi + omega * dt) - cos(psi)) + (v*dt/omega) * sin(psi + omega * dt);
        F26 = 0;
    else
        % Caso particolare quando omega è 0 (il veicolo si muove in linea retta)
        F11 = 1;
        F12 = 0;
        F13 = -v*dt*sin(psi);
        F14 = dt*cos(psi);
        F15 = 0;
        F16 = 0;

        F21 = 0;
        F22 = 1;
        F23 = v*dt*cos(psi);
        F24 = dt*sin(psi);
        F25 = 0;
        F26 = 0;
    end

    F31 = 0;
    F32 = 0;
    F33 = 1;
    F34 = 0;
    F35 = dt;
    F36 = 0;

    F41 = 0;
    F42 = 0;
    F43 = 0;
    F44 = 1;
    F45 = 0;
    F46 = dt;

    F51 = 0;
    F52 = 0;
    F53 = 0;
    F54 = 0;
    F55 = 1;
    F56 = 0;

    F61 = 0;
    F62 = 0;
    F63 = 0;
    F64 = 0;
    F65 = 0;
    F66 = 1;

    % Costruzione della matrice Jacobiana
    F = [F11, F12, F13, F14, F15, F16;
         F21, F22, F23, F24, F25, F26;
         F31, F32, F33, F34, F35, F36;
         F41, F42, F43, F44, F45, F46;
         F51, F52, F53, F54, F55, F56;
         F61, F62, F63, F64, F65, F66];
end
