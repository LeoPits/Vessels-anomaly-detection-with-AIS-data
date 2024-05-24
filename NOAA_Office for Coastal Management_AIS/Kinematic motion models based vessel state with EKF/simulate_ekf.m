% Parametri di simulazione
rng(6);
Ts =60;       % Intervallo di tempo (secondi)
T =Ts*200;         % Tempo totale di simulazione (secondi)
N = T / Ts;     % Numero di passi temporali

% Stato iniziale [px, py, v, heading,turn rate,acc]
x = [0; 0; 0.5; pi/4;0;0];  

% Inizializzazione delle variabili per memorizzare i risultati
trajectory = zeros(6, N);
trajectory(:, 1) = x;

for k = 2:N
    x = stateModel(x, Ts);
    trajectory(:, k) = x;
end

% Plot della traiettoria
figure;
plot(trajectory(1, :), trajectory(2, :), 'b-', 'LineWidth', 2);
xlabel('Posizione X (m)');
ylabel('Posizione Y (m)');
title('Traiettoria della Nave con Modello ');
grid on;
axis equal;
%%

ekf = extendedKalmanFilter('HasAdditiveMeasurementNoise',true);
ekf.StateTransitionFcn = @(x,u)stateModel(x,Ts);
ekf.MeasurementFcn =  @(x)measureModel(x);
ekf.StateTransitionJacobianFcn =@(x,u)stateTransitionJacobianFcn(x,Ts);

ekf.StateCovariance=diag([0,0,0,0,0,0]);  % Covarianza iniziale
ekf.ProcessNoise=diag([1e-5,1e-5,1e-5,1e-5,1e-12,1e-12]);
ekf.MeasurementNoise= diag([0,0,0,0]);
% Stato iniziale [px, py, v, heading,turn rate,acc]
x = [0; 0; 0.5; pi/4;0;0];  
ekf.State = x;

%%
trajectory_true = zeros(6, N);
trajectory_est = zeros(6, N);
anomalies = zeros(1, N);

trajectory_true(:, 1) = x;
trajectory_est(:, 1) = ekf.State;
% Loop di simulazione
for k = 2:N
    % % Cambio di direzione improvviso
    if k == 60
        x(3) = 0.7; % Cambio dell'orientamento
        x(4) = -pi/4; % Cambio dell'orientamento

    end
    % Aggiungi rumore di processo
    process_noise = sqrt(ekf.ProcessNoise) * randn(6, 1);
    x = stateModel(x, Ts) + process_noise;    % % Simula il movimento vero
    trajectory_true(:, k) = x;
    % Simula misurazione con rumore
    z = measureModel(x) + sqrt(ekf.MeasurementNoise) * randn(4, 1);
    % EKF Predict
    predict(ekf,[]);
    % EKF Correct
    correct(ekf, z);
    % Memorizza lo stato stimato
    trajectory_est(:, k) = ekf.State;

end
%%
% Plot della traiettoria
figure;
plot(trajectory_true(1, :), trajectory_true(2, :), 'g-', 'LineWidth', 2);
hold on;
plot(trajectory_est(1, :), trajectory_est(2, :), 'b--', 'LineWidth', 2);
xlabel('Posizione X (m)');
ylabel('Posizione Y (m)');
title('Traiettoria della Nave con EKF e Rilevamento Anomalie');
legend('Traiettoria Vera', 'Traiettoria Stimata');
grid on;
axis equal;


%%
figure
hold on
plot(trajectory_est(5, :), 'r--', 'LineWidth', 2);
plot(trajectory_est(6, :), 'b--', 'LineWidth', 2);
xlabel('Passo Temporale');
grid on;