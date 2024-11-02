import casadi.*

% Parametri del modello della nave
dt = 0.1; % Passo temporale
T = 50; % Numero di passi temporali

% Stato iniziale
x0 = [0; 0; 1; pi/4; 0.1; 0]; % [px; py; v; heading; omega; a]

% Matrici di covarianza
Q = 0.01 * eye(6);  % Covarianza del rumore di processo
R = 0.1 * eye(2);  % Covarianza del rumore di misura

% Inizializzazione degli array di stato e di uscita
X_true = zeros(6, T);
Y = zeros(2, T);

% Simulazione dei dati di stato e di uscita
X_true(:, 1) = x0;
for t = 2:T
    % Estrazione delle variabili di stato
    px = X_true(1, t-1);
    py = X_true(2, t-1);
    v = X_true(3, t-1);
    heading = X_true(4, t-1);
    omega = X_true(5, t-1);
    a = X_true(6, t-1);

    % Calcolo del prossimo stato
    px_next = px + v * dt * cos(heading);
    py_next = py + v * dt * sin(heading);
    v_next = v + a * dt;
    heading_next = heading + omega * dt;
    omega_next = omega;
    a_next = a;
    
    % Stato successivo
    X_true(:, t) = [px_next; py_next; v_next; heading_next; omega_next; a_next];
    
    % Uscita misurata con aggiunta di rumore
    Y(:, t) = [px_next; py_next] + sqrt(R) * randn(2, 1);
end

% Parametri MHE
N = 10; % Lunghezza della finestra temporale
x_hat = x0; % Stato iniziale per MHE

% Memorizzazione dei risultati
X_hat = zeros(6, T);

% Definizione delle variabili CASADI
x = SX.sym('x', 6, N+1); % Stato
u = SX.sym('u', 2, N); % Uscita misurata

% Funzione di transizione di stato
x_next = SX.zeros(6, 1);
x_next(1) = x(1) + x(3) * dt * cos(x(4));
x_next(2) = x(2) + x(3) * dt * sin(x(4));
x_next(3) = x(3) + x(6) * dt;
x_next(4) = x(4) + x(5) * dt;
x_next(5) = x(5);
x_next(6) = x(6);

f = Function('f', {x(:,1)}, {x_next});

% Definizione della funzione obiettivo
J = 0;
g = [];

for i = 1:N
    y_hat = [x(1, i); x(2, i)];
    J = J + (u(:, i) - y_hat)' * R * (u(:, i) - y_hat) + (x(:, i+1) - f(x(:, i)))' * Q * (x(:, i+1) - f(x(:, i)));
end

% Funzione obiettivo
obj = Function('obj', {x, u}, {J});
nlp = struct('x', reshape(x, [], 1), 'f', J, 'g', g);

% Opzioni del solver
opts = struct;
opts.ipopt.print_level = 0;
opts.print_time = 0;

% Solver
solver = nlpsol('solver', 'ipopt', nlp, opts);

% Implementazione di MHE con CASADI
for k = 1:(T - N)
    % Definizione del problema di ottimizzazione
    y_hat = Y(:, k:k+N-1); % Uscite misurate
    
    % Risoluzione del problema di ottimizzazione
    sol = solver('x0', reshape(x_hat, [], 1), 'lbg', 0, 'ubg', 0, 'p', y_hat);
    
    % Aggiornamento dello stato stimato
    x_hat = full(sol.x);
    x_hat = reshape(x_hat, 6, N+1);
    
    % Memorizzazione dello stato stimato
    X_hat(:, k) = x_hat(:, 1);
end

% Visualizzazione dei risultati
figure;
plot(X_true(1, :), X_true(2, :), 'b', 'LineWidth', 2); hold on;
plot(X_hat(1, :), X_hat(2, :), 'r--', 'LineWidth', 2);
legend('Traiettoria reale', 'Traiettoria stimata');
xlabel('Posizione X');
ylabel('Posizione Y');
title('Stima della traiettoria della nave con MHE usando CASADI');
grid on;
