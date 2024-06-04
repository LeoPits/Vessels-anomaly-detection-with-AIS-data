%% Tracking IMM

% Define simulation parameters
dt = 1; % Time step (s)
numSteps = 100; % Number of simulation steps
sigma = 0.1; % Process noise standard deviation

% Define the state transition matrix (F) for the Constant Velocity Model (CVM)
F_CVM = [1 0 dt 0;
         0 1 0 dt;
         0 0 1 0;
         0 0 0 1];
     
% Define the state transition matrix (F) for the Constant Turn Model (CTM)
omega = 0.1; % Constant turn rate
F_CTM = [1 0 sin(omega*dt)/omega -(1-cos(omega*dt))/omega;
         0 1 (1-cos(omega*dt))/omega sin(omega*dt)/omega;
         0 0 cos(omega*dt) -sin(omega*dt);
         0 0 sin(omega*dt) cos(omega*dt)];
     
% Define the measurement matrix (H)
H = [1 0 0 0;
     0 1 0 0];
 
% Define process noise covariance (Q)
Q_CVM = sigma^2 * [dt^4/4 0 dt^3/2 0; 
                   0 dt^4/4 0 dt^3/2; 
                   dt^3/2 0 dt^2 0; 
                   0 dt^3/2 0 dt^2];
Q_CTM = sigma^2 * eye(4);

% Define measurement noise covariance (R)
R = sigma^2 * eye(2);

% Define the initial state
x0 = [0; 0; 1; 1];

% Define the initial state covariance
P0 = 1 * eye(4);

% Define the initial mode probabilities
mu = [0.5; 0.5];

% Define transition probabilities
P = [0.9 0.1; 0.1 0.9];

% Number of models
numModels = 2;

% Initialize the state estimates and covariances for each model
x_estimates = repmat(x0, 1, numModels);
P_estimates = repmat(P0, 1, 1, numModels);

% Storage for results
x_est = zeros(4, numSteps);
x_true = zeros(4, numSteps);
z = zeros(2, numSteps);
model_probabilities = zeros(numModels, numSteps);

% Initial state
x_true(:,1) = x0;

% Simulate true trajectory
for k = 2:numSteps
    % True state (use CTM for simulation)
    x_true(:,k) = F_CTM * x_true(:,k-1) + mvnrnd(zeros(4,1), Q_CTM)';
    % Measurement
    z(:,k) = H * x_true(:,k) + mvnrnd(zeros(2,1), R)';
end

% IMM filter loop
for k = 1:numSteps
    % Interaction (mixing)
    x_mix = zeros(4, numModels);
    P_mix = zeros(4, 4, numModels);
    for i = 1:numModels
        for j = 1:numModels
            x_mix(:,i) = x_mix(:,i) + P(j,i) * x_estimates(:,j);
        end
        for j = 1:numModels
            P_mix(:,:,i) = P_mix(:,:,i) + P(j,i) * (P_estimates(:,:,j) + (x_estimates(:,j) - x_mix(:,i)) * (x_estimates(:,j) - x_mix(:,i))');
        end
    end

    % Model-specific Kalman filtering
    x_predict = zeros(4, numModels);
    P_predict = zeros(4, 4, numModels);
    L = zeros(numModels, 1);
    for i = 1:numModels
        if i == 1
            F = F_CVM;
            Q = Q_CVM;
        else
            F = F_CTM;
            Q = Q_CTM;
        end
        % Prediction
        x_predict(:,i) = F * x_mix(:,i);
        P_predict(:,:,i) = F * P_mix(:,:,i) * F' + Q;
        % Update
        y = z(:,k) - H * x_predict(:,i);
        S = H * P_predict(:,:,i) * H' + R;
        K = P_predict(:,:,i) * H' / S;
        x_estimates(:,i) = x_predict(:,i) + K * y;
        P_estimates(:,:,i) = (eye(4) - K * H) * P_predict(:,:,i);
        % Likelihood
        L(i) = exp(-0.5 * (y' / S * y)) / sqrt(det(2 * pi * S));
    end

    % Model probability update
    c = L' * mu;
    mu = (L .* mu) / c;

    % State and covariance combination
    x_est(:,k) = x_estimates * mu;
    P_combined = zeros(4, 4);
    for i = 1:numModels
        P_combined = P_combined + mu(i) * (P_estimates(:,:,i) + (x_estimates(:,i) - x_est(:,k)) * (x_estimates(:,i) - x_est(:,k))');
    end

    % Store model probabilities
    model_probabilities(:,k) = mu;
end

% Plot results
figure;
subplot(2,1,1);
plot(x_true(1,:), x_true(2,:), 'g-', 'LineWidth', 2); hold on;
plot(z(1,:), z(2,:), 'r.', 'MarkerSize', 10);
plot(x_est(1,:), x_est(2,:), 'b-', 'LineWidth', 2);
legend('True Position', 'Measurements', 'Estimated Position');
title('Position Tracking using IMM Filter');
xlabel('X Position');
ylabel('Y Position');

subplot(2,1,2);
plot(model_probabilities(1,:), 'r-', 'LineWidth', 2); hold on;
plot(model_probabilities(2,:), 'b-', 'LineWidth', 2);
legend('Mode Probability CVM', 'Mode Probability CTM');
title('Mode Probabilities');
xlabel('Time Step');
ylabel('Probability');
%% Generate missing data and estimate
% Define simulation parameters
dt = 1; % Time step (s)
numSteps = 100; % Number of simulation steps
sigma = 0.1; % Process noise standard deviation

% Define the state transition matrix (F) for the Constant Velocity Model (CVM)
F_CVM = [1 0 dt 0;
         0 1 0 dt;
         0 0 1 0;
         0 0 0 1];
     
% Define the state transition matrix (F) for the Constant Turn Model (CTM)
omega = 0.1; % Constant turn rate
F_CTM = [1 0 sin(omega*dt)/omega -(1-cos(omega*dt))/omega;
         0 1 (1-cos(omega*dt))/omega sin(omega*dt)/omega;
         0 0 cos(omega*dt) -sin(omega*dt);
         0 0 sin(omega*dt) cos(omega*dt)];
     
% Define the measurement matrix (H)
H = [1 0 0 0;
     0 1 0 0];
 
% Define process noise covariance (Q)
Q_CVM = sigma^2 * [dt^4/4 0 dt^3/2 0; 
                   0 dt^4/4 0 dt^3/2; 
                   dt^3/2 0 dt^2 0; 
                   0 dt^3/2 0 dt^2];
Q_CTM = sigma^2 * eye(4);

% Define measurement noise covariance (R)
R = sigma^2 * eye(2);

% Define the initial state
x0 = [0; 0; 1; 1];

% Define the initial state covariance
P0 = 1 * eye(4);

% Define the initial mode probabilities
mu = [0.5; 0.5];

% Define transition probabilities
P = [0.9 0.1; 0.1 0.9];

% Number of models
numModels = 2;

% Initialize the state estimates and covariances for each model
x_estimates = repmat(x0, 1, numModels);
P_estimates = repmat(P0, 1, 1, numModels);

% Storage for results
x_est = zeros(4, numSteps);
x_true = zeros(4, numSteps);
z = NaN(2, numSteps); % Initialize with NaN to represent missing measurements
model_probabilities = zeros(numModels, numSteps);

% Initial state
x_true(:,1) = x0;

% Simulate true trajectory and generate measurements with missing data
for k = 2:numSteps
    % True state (use CTM for simulation)
    x_true(:,k) = F_CTM * x_true(:,k-1) + mvnrnd(zeros(4,1), Q_CTM)';
    % Randomly decide if the measurement is missing (e.g., 20% missing data)
    if rand > 0.6
        z(:,k) = H * x_true(:,k) + mvnrnd(zeros(2,1), R)';
    end
end

% IMM filter loop
for k = 1:numSteps
    % Interaction (mixing)
    x_mix = zeros(4, numModels);
    P_mix = zeros(4, 4, numModels);
    for i = 1:numModels
        for j = 1:numModels
            x_mix(:,i) = x_mix(:,i) + P(j,i) * x_estimates(:,j);
        end
        for j = 1:numModels
            P_mix(:,:,i) = P_mix(:,:,i) + P(j,i) * (P_estimates(:,:,j) + (x_estimates(:,j) - x_mix(:,i)) * (x_estimates(:,j) - x_mix(:,i))');
        end
    end

    % Model-specific Kalman filtering
    x_predict = zeros(4, numModels);
    P_predict = zeros(4, 4, numModels);
    L = zeros(numModels, 1);
    for i = 1:numModels
        if i == 1
            F = F_CVM;
            Q = Q_CVM;
        else
            F = F_CTM;
            Q = Q_CTM;
        end
        % Prediction
        x_predict(:,i) = F * x_mix(:,i);
        P_predict(:,:,i) = F * P_mix(:,:,i) * F' + Q;
        % Update if measurement is available
        if ~isnan(z(1,k))
            y = z(:,k) - H * x_predict(:,i);
            S = H * P_predict(:,:,i) * H' + R;
            K = P_predict(:,:,i) * H' / S;
            x_estimates(:,i) = x_predict(:,i) + K * y;
            P_estimates(:,:,i) = (eye(4) - K * H) * P_predict(:,:,i);
            % Likelihood
            L(i) = exp(-0.5 * (y' / S * y)) / sqrt(det(2 * pi * S));
        else
            x_estimates(:,i) = x_predict(:,i);
            P_estimates(:,:,i) = P_predict(:,:,i);
            L(i) = 1; % Neutral likelihood when no measurement
        end
    end

    % Model probability update
    c = L' * mu;
    mu = (L .* mu) / c;

    % State and covariance combination
    x_est(:,k) = x_estimates * mu;
    P_combined = zeros(4, 4);
    for i = 1:numModels
        P_combined = P_combined + mu(i) * (P_estimates(:,:,i) + (x_estimates(:,i) - x_est(:,k)) * (x_estimates(:,i) - x_est(:,k))');
    end

    % Store model probabilities
    model_probabilities(:,k) = mu;
end

% Plot results
figure;
subplot(2,1,1);
plot(x_true(1,:), x_true(2,:), 'g-', 'LineWidth', 2); hold on;
plot(z(1,:), z(2,:), 'r.', 'MarkerSize', 10);
plot(x_est(1,:), x_est(2,:), 'b-', 'LineWidth', 2);
legend('True Position', 'Measurements', 'Estimated Position');
title('Position Tracking using IMM Filter with Missing Data');
xlabel('X Position');
ylabel('Y Position');

subplot(2,1,2);
plot(model_probabilities(1,:), 'r-', 'LineWidth', 2); hold on;
plot(model_probabilities(2,:), 'b-', 'LineWidth', 2);
legend('Mode Probability CVM', 'Mode Probability CTM');
title('Mode Probabilities');
xlabel('Time Step');
ylabel('Probability');
%% To simulate an AIS-like scenario where the transmission times are irregular, we can modify the simulation to include random time intervals between measurements. This involves:
% Define simulation parameters
dt = 1; % Average time step (s)
numSteps = 100; % Number of simulation steps
sigma = 0.1; % Process noise standard deviation

% Define the state transition matrix (F) for the Constant Velocity Model (CVM)
F_CVM = @(dt) [1 0 dt 0;
               0 1 0 dt;
               0 0 1 0;
               0 0 0 1];
     
% Define the state transition matrix (F) for the Constant Turn Model (CTM)
omega = 0.1; % Constant turn rate
F_CTM = @(dt) [1 0 sin(omega*dt)/omega -(1-cos(omega*dt))/omega;
               0 1 (1-cos(omega*dt))/omega sin(omega*dt)/omega;
               0 0 cos(omega*dt) -sin(omega*dt);
               0 0 sin(omega*dt) cos(omega*dt)];
     
% Define the measurement matrix (H)
H = [1 0 0 0;
     0 1 0 0];
 
% Define process noise covariance (Q)
Q_CVM = @(dt) sigma^2 * [dt^4/4 0 dt^3/2 0; 
                         0 dt^4/4 0 dt^3/2; 
                         dt^3/2 0 dt^2 0; 
                         0 dt^3/2 0 dt^2];
Q_CTM = @(dt) sigma^2 * eye(4);

% Define measurement noise covariance (R)
R = sigma^2 * eye(2);

% Define the initial state
x0 = [0; 0; 1; 1];

% Define the initial state covariance
P0 = 1 * eye(4);

% Define the initial mode probabilities
mu = [0.5; 0.5];

% Define transition probabilities
P = [0.9 0.1; 0.1 0.9];

% Number of models
numModels = 2;

% Initialize the state estimates and covariances for each model
x_estimates = repmat(x0, 1, numModels);
P_estimates = repmat(P0, 1, 1, numModels);

% Storage for results
x_est = zeros(4, numSteps);
x_true = zeros(4, numSteps);
z = zeros(2, numSteps); % Measurement matrix
time_steps = zeros(1, numSteps); % Time intervals
model_probabilities = zeros(numModels, numSteps);

% Initial state
x_true(:,1) = x0;

% Simulate true trajectory and generate measurements with irregular time intervals
time_steps(1) = dt;
for k = 2:numSteps
    % Generate a random time step
    time_steps(k) = dt + 2*randn; % Mean dt with some noise
    if time_steps(k) <= 0
        time_steps(k) = dt; % Ensure positive time step
    end
    % True state (use CTM for simulation)
    x_true(:,k) = F_CTM(time_steps(k)) * x_true(:,k-1) + mvnrnd(zeros(4,1), Q_CTM(time_steps(k)))';
    % Measurement
    z(:,k) = H * x_true(:,k) + mvnrnd(zeros(2,1), R)';
end

% IMM filter loop
for k = 1:numSteps
    % Interaction (mixing)
    x_mix = zeros(4, numModels);
    P_mix = zeros(4, 4, numModels);
    for i = 1:numModels
        for j = 1:numModels
            x_mix(:,i) = x_mix(:,i) + P(j,i) * x_estimates(:,j);
        end
        for j = 1:numModels
            P_mix(:,:,i) = P_mix(:,:,i) + P(j,i) * (P_estimates(:,:,j) + (x_estimates(:,j) - x_mix(:,i)) * (x_estimates(:,j) - x_mix(:,i))');
        end
    end

    % Model-specific Kalman filtering
    x_predict = zeros(4, numModels);
    P_predict = zeros(4, 4, numModels);
    L = zeros(numModels, 1);
    for i = 1:numModels
        if i == 1
            F = F_CVM(time_steps(k));
            Q = Q_CVM(time_steps(k));
        else
            F = F_CTM(time_steps(k));
            Q = Q_CTM(time_steps(k));
        end
        % Prediction
        x_predict(:,i) = F * x_mix(:,i);
        P_predict(:,:,i) = F * P_mix(:,:,i) * F' + Q;
        % Update
        y = z(:,k) - H * x_predict(:,i);
        S = H * P_predict(:,:,i) * H' + R;
        K = P_predict(:,:,i) * H' / S;
        x_estimates(:,i) = x_predict(:,i) + K * y;
        P_estimates(:,:,i) = (eye(4) - K * H) * P_predict(:,:,i);
        % Likelihood
        L(i) = exp(-0.5 * (y' / S * y)) / sqrt(det(2 * pi * S));
    end

    % Model probability update
    c = L' * mu;
    mu = (L .* mu) / c;

    % State and covariance combination
    x_est(:,k) = x_estimates * mu;
    P_combined = zeros(4, 4);
    for i = 1:numModels
        P_combined = P_combined + mu(i) * (P_estimates(:,:,i) + (x_estimates(:,i) - x_est(:,k)) * (x_estimates(:,i) - x_est(:,k))');
    end

    % Store model probabilities
    model_probabilities(:,k) = mu;
end

% Plot results
figure;
subplot(2,1,1);
plot(x_true(1,:), x_true(2,:), 'g-', 'LineWidth', 2); hold on;
plot(z(1,:), z(2,:), 'r.', 'MarkerSize', 10);
plot(x_est(1,:), x_est(2,:), 'b-', 'LineWidth', 2);
legend('True Position', 'Measurements', 'Estimated Position');
title('Position Tracking using IMM Filter with Irregular Transmission Times');
xlabel('X Position');
ylabel('Y Position');

subplot(2,1,2);
plot(model_probabilities(1,:), 'r-', 'LineWidth', 2); hold on;
plot(model_probabilities(2,:), 'b-', 'LineWidth', 2);
legend('Mode Probability CVM', 'Mode Probability CTM');
title('Mode Probabilities');
xlabel('Time Step');
ylabel('Probability');
