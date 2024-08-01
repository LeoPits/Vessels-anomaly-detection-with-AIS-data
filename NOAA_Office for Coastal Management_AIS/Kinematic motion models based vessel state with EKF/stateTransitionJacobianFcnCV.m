function J = stateTransitionJacobianFcnCV(state, dt)
    % Calcola la Jacobiana della funzione di transizione del modello di stato
    % stateModelCV
    
    % Estrai le variabili di stato
    px = state(1);        % Posizione x corrente
    py = state(2);        % Posizione y corrente
    vx = state(3);        % Velocità x corrente
    vy = state(4);        % Velocità y corrente
    heading = state(5);   % Orientamento corrente
    omega = state(6);     % Velocità angolare corrente
    a = state(7);         % Accelerazione corrente
    
    % Calcola la prossima posizione utilizzando la velocità costante
    px_next = px + vx * dt;
    py_next = py + vy * dt;
    vx_next = vx;
    vy_next = vy;
    heading_next = heading;
    omega_next = omega;
    a_next = a;
    
    % Calcola la Jacobiana della funzione di transizione
    J = [dt 0 1 0 0 0 0; 
         0 dt 0 1 0 0 0; 
         0 0 0 0 0 0 0; 
         0 0 0 0 0 0 0; 
         0 0 0 0 0 0 0; 
         0 0 0 0 0 0 0; 
         0 0 0 0 0 0 0];
end