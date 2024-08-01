%% LEARNING MOTION PATTERNS IN AIS DATA wiht EKF
clear;
close all;
clc;
% 
%info=parquetinfo('Dati Danimarca pre-processati da AIA\denmark_2023-06-05_downsampled.parquet')
%select only one ship from the non-interpolated dataset.
%  create a row filter base uniique identifier "vid"
rf = rowfilter(["vid"]);% seleziono solo una 
%rf2 = (rf.vid == 205096000);
rf2 = (rf.vid == 244090800);
%rf2 = (rf.vid == 258009780);
%extrac dataset filterd
dataset = parquetread('Dati Danimarca pre-processati da AIA\denmark_2023-06-05_point-based.parquet',RowFilter=rf2);
% resample data (1sec) and clean
resample_dataset_clean = preprocess_data(dataset);
% figure
% % Create a geographical plot
% geoplot(resample_dataset_clean.lat, resample_dataset_clean.lon, 'o-')
% title('Sample Geographical Plot')
%%
% Convert timetable to table
resample_dataset_clean = timetable2table(resample_dataset_clean);
% Use the 'utmzone' function to obtain the UTM zone
% The UTM ( Universal Transverse Mercator ) coordinate system divides the world into 
% sixty north-south zones, each 6 degrees of longitude wide. UTM zones are numbered 
% consecutively beginning with Zone 1, which includes the westernmost point of Alaska,
% and progress eastward to Zone 19, which includes Maine.
% LAT,LON ->utmX,utmY I follow paper "Learning motion patterns in AIS data
% and detecting anomalous vessel behavior"
zone = utmzone(resample_dataset_clean.lat, resample_dataset_clean.lon);
% Obtain the UTM projection map for the calculated zone
utmstruct = defaultm('utm');
utmstruct.zone = zone;
utmstruct.geoid = wgs84Ellipsoid;
utmstruct = defaultm(utmstruct);

% Convert all lat/lon coordinates to UTM
[utmX, utmY] = projfwd(utmstruct, resample_dataset_clean.lat, resample_dataset_clean.lon);
% Convert UTM coordinates back to lat/lon
[lat, lon] = projinv(utmstruct, utmX, utmY);

% Create a figure for plotting
figure
% Create a plot of UTM coordinates
plot(utmX, utmY, 'o-')
grid on
title('Sample Geographical Plot')

% Create another figure for plotting
figure
% Create a geographical plot
geoplot(lat, lon, 'o-')
title('Sample Geographical Plot')

% Assuming 'temp' is a table and 'BaseDateTime' is a column in the 'dd/mm/yyyy HH:MM' format
% Convert 'BaseDateTime' column to datetime format
resample_dataset_clean.t = datetime(resample_dataset_clean.t,'InputFormat','dd/MM/yyyy HH:mm');

% Convert datetime to seconds since epoch (Unix timestamp)
d = posixtime(resample_dataset_clean.t);

% Calculate time elapsed since the first timestamp
Tsec = d - d(1);
% Assuming 'Tsec' is your array of timestamps in seconds
diff_Tsec = diff(Tsec);

% Find the most common difference, which will be your sample time Ts
Ts = mode(diff_Tsec);

%%
% state_vector=[utmX,utmY,resample_dataset_clean.sog, resample_dataset_clean.heading];

[x,y]=normalize_utm(utmX,utmY);
vx=resample_dataset_clean.sog.*cos(resample_dataset_clean.heading);
vy=resample_dataset_clean.sog.*sin(resample_dataset_clean.heading);
state_vector=[x,y,vx, vy];
state_vector=state_vector';
numSteps=length(state_vector);
%%
dt=1;
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

% Define the initial state covariance
P0 = 1 * eye(4);

% Define the initial mode probabilities
mu = [0.5; 0.5];

% Define transition probabilities
P = [0.9 0.1; 0.1 0.9];

% Number of models
numModels = 2;

% Initialize the state estimates and covariances for each model
x_estimates = repmat(state_vector(:,1), 1, numModels);
P_estimates = repmat(P0, 1, 1, numModels);

% Storage for results
x_est = zeros(4, numSteps);
x_true = zeros(4, numSteps);
z = zeros(2, numSteps);
model_probabilities = zeros(numModels, numSteps);

% Initial state
x_true = state_vector;

% Simulate true trajectory
for k = 1:numSteps
    % Measurement
    z(:,k) = H * x_true(:,k);
end
%%

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