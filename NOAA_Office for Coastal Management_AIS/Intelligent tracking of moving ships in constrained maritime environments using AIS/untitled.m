%% read DATA
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
%%
% Use the 'utmzone' function to obtain the UTM zone
% The UTM ( Universal Transverse Mercator ) coordinate system divides the world into 
% sixty north-south zones, each 6 degrees of longitude wide. UTM zones are numbered 
% consecutively beginning with Zone 1, which includes the westernmost point of Alaska,
% and progress eastward to Zone 19, which includes Maine.
% LAT,LON ->utmX,utmY I follow paper "Learning motion patterns in AIS data
% and detecting anomalous vessel behavior"
zone = utmzone(dataset.lat, dataset.lon);
% Obtain the UTM projection map for the calculated zone
utmstruct = defaultm('utm');
utmstruct.zone = zone;
utmstruct.geoid = wgs84Ellipsoid;
utmstruct = defaultm(utmstruct);

% Convert all lat/lon coordinates to UTM
[utmX, utmY] = projfwd(utmstruct, dataset.lat, dataset.lon);
% Convert UTM coordinates back to lat/lon
[lat, lon] = projinv(utmstruct, utmX, utmY);
%% state vector 
state_vector=[utmX,utmY,dataset.sog, dataset.heading];
state_vector=state_vector';
numSteps=length(state_vector);
dt = 1; % Average time step (s)
%%
% Define models for the IMM filter
% Example: Constant Velocity (CV) and Constant Turn (CT) models
F1 = [1 dt 0 0;
      0 1  0 0;
      0 0  1 dt;
      0 0  0 1];
  
Q1 = diag([0.1, 0.1, 0.1, 0.1]);

F2 = [1 dt 0 0;
      0 1  0 0;
      0 0  1 dt;
      0 0  0 1];
  
Q2 = diag([0.1, 0.1, 0.1, 0.1]);

H = [1 0 0 0;
     0 0 1 0];

R = diag([0.5, 0.5]);

% Initial state and covariance
x0 = state_vector(:,1);
P0 = diag([1, 1, 1, 1]);

% Model transition matrix
MU = [0.95 0.05;
      0.05 0.95];

% Initial mode probabilities
mu0 = [0.5; 0.5];
%%
% Initialize IMM filter
models = {struct('A', F1, 'Q', Q1), struct('A', F2, 'Q', Q2)};
imm = imm_filter(mu0, models, H, R, x0, P0);

% Run the IMM filter through the dataset
estimated_states = zeros(size(state_vector));
for k = 1:numSteps
    z = state_vector(:, k);
    [x_est, P_est] = imm_step(imm, z);
    estimated_states(:, k) = x_est;
end

%%
function imm = imm_filter(mu0, models, H, R, x0, P0)
    imm.mu = mu0;
    imm.models = models;
    imm.H = H;
    imm.R = R;
    imm.x = repmat(x0, 1, length(models));
    imm.P = repmat(P0, [1, 1, length(models)]);
end
function [x_est, P_est] = imm_step(imm, z)
    % Prediction and update for each model
    for i = 1:length(imm.models)
        % Prediction
        F = imm.models{i}.A;
        Q = imm.models{i}.Q;
        x_pred = F * imm.x(:, i);
        P_pred = F * imm.P(:, :, i) * F' + Q;

        % Update
        y = z - imm.H * x_pred;
        S = imm.H * P_pred * imm.H' + imm.R;
        K = P_pred * imm.H' / S;
        imm.x(:, i) = x_pred + K * y;
        imm.P(:, :, i) = (eye(size(K, 1)) - K * imm.H) * P_pred;
    end

    % Combine estimates
    mu_pred = imm.mu' * cellfun(@(model) model.A * imm.x, imm.models, 'UniformOutput', false);
    x_est = sum(mu_pred .* imm.x, 2);
    P_est = sum(mu_pred .* imm.P, 3);
end
