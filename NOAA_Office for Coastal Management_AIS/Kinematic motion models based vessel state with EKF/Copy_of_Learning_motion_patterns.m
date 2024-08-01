%% LEARNING MOTION PATTERNS IN AIS DATA wiht EKF
clear;
close all;
clc;
% 
info=parquetinfo('Dati Danimarca pre-processati da AIA\denmark_2023-06-05_downsampled.parquet')
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
state_vector=[utmX,utmY,resample_dataset_clean.sog, resample_dataset_clean.heading];
state_vector=state_vector';
N=length(state_vector);
state_vector=[state_vector;zeros(3,N)];

% Create an extended Kalman filter object
ekf = extendedKalmanFilter('HasAdditiveMeasurementNoise',true);
ekf.StateTransitionFcn = @(x,u)CTRV_ext_StateFcn(x,Ts);
ekf.MeasurementFcn =  @(x,u)measureModel(x);
ekf.StateCovariance=diag([1e-5,1e-5,1e-5,1e-5,1e-5,1e-5,1e-5]);  % Covarianza iniziale
ekf.ProcessNoise=diag([1e-5,1e-5,1e-5,1e-5,1e-8,1e-8,1e-8]);
ekf.MeasurementNoise= diag([0,0,0,0]);
ekf.State = state_vector(:,1);


%% Generate true states and measurements
for i = 2:N
    if i ~= 1
        trueStates(:,i) = CTRV_ext_StateFcn(state_vector(:,i-1),Ts) +sqrt(ekf.ProcessNoise)*randn(7,1);  
    end
    measurements(:,i) = measureModel(state_vector(:,i)) + sqrt(ekf.MeasurementNoise)*randn(4,1);
end
%%  Perform prediction and correction steps
for i=2:N
    correct(ekf,measurements(:,i),[]);
    predict(ekf,measurements(:,i));
    estimateStates(:,i)=ekf.State;
end
% Plot the results
figure
hold on
plot(measurements(2,2),measurements(1,2),"r*",'LineWidth',8,DisplayName="Measure star")
plot(measurements(2,2:end),measurements(1,2:end),"r",'LineWidth',3,DisplayName="Measure")
plot(estimateStates(2,2:end),estimateStates(1,2:end),"b--",'LineWidth',3,DisplayName="Estimate")
legend(Location="northwest")
grid on
xlabel("x (m)")
ylabel("y (m)")
title("True Trajectory")
axis square

%% Plot the true trajectory and estimated trajectory in 3D
figure
hold on
plot3(Tsec(2:end)/(3600),measurements(2,2:end),measurements(1,2:end),"b",'LineWidth',4,DisplayName="Measure")
plot3(Tsec(2:end)/(3600),estimateStates(2,2:end),estimateStates(1,2:end),"r--",'LineWidth',2,DisplayName="Estimate")
legend(Location="northwest")
grid on
xlabel("$t [hour]$",'Interpreter','latex')
ylabel("$x [m]$",'Interpreter','latex')
zlabel("$y [m]$",'Interpreter','latex')
title(" Plot the true trajectory and estimated trajectory in 3D")
axis square
%% Plot estimated turn rate and acceleration

figure
hold on
subplot 311
plot(Tsec(2:end)./(3600),estimateStates(5,2:end),"b",'LineWidth',4,DisplayName="turn rate")
xlabel("$t [hour]$",'Interpreter','latex')
ylabel("turn rate $(\frac{rad^2}{sec})$",'Interpreter','latex')

grid on
legend(Location="northwest")
title(" Plot estimated turn rate and acceleration")

subplot 312
plot(Tsec(2:end)/(3600),estimateStates(6,2:end),"r",'LineWidth',4,DisplayName="acc v")
xlabel("$t [hour]$",'Interpreter','latex')
ylabel("acceleration  $(\frac{m^2}{sec})$",'Interpreter','latex')
legend(Location="northwest")
grid on

subplot 313
plot(Tsec(2:end)/(3600),estimateStates(7,2:end),"r",'LineWidth',4,DisplayName="acc turn rate")
xlabel("$t [hour]$",'Interpreter','latex')
ylabel("acceleration  $(\frac{m^2}{sec})$",'Interpreter','latex')
legend(Location="northwest")
grid on

%%  Plot estimated speed against true speed

figure
hold on
plot(Tsec(2:end)./(3600),measurements(3,2:end),"r",'LineWidth',4,DisplayName="speed true")
plot(Tsec(2:end)./(3600),estimateStates(3,2:end),"b--",'LineWidth',1,DisplayName="speed est")

xlabel("$t [hour]$",'Interpreter','latex')
ylabel("speed  $(\frac{m}{sec})$",'Interpreter','latex')
legend(Location="northwest")
grid on
%% Plot estimated heading against true heading
figure
hold on
plot(Tsec(2:end)./(3600),measurements(4,2:end),"r",'LineWidth',4,DisplayName="heading true")
plot(Tsec(2:end)./(3600),estimateStates(4,2:end),"b--",'LineWidth',4,DisplayName="heading est")
xlabel("$t [hour]$",'Interpreter','latex')
ylabel("heading  $[rad]$",'Interpreter','latex')
legend(Location="northwest")
title('Plot estimated heading against true heading')
grid on
%% Convert UTM coordinates back to lat/lon
[lat_e, lon_e] = projinv(utmstruct, estimateStates(1,2:end), estimateStates(2,2:end));
figure
% Create a geographical plot
geoplot(lat_e, lon_e, 'o-')
title('Plot the estimated trajectory on a geographical map')
%%
acc_raw=diff(resample_dataset_clean.sog);
figure
hold on
plot(Tsec(2:end)./(3600),acc_raw,"b--",'LineWidth',4,DisplayName="acc_raw")
plot(Tsec(2:end)/(3600),estimateStates(6,2:end),"r",'LineWidth',4,DisplayName="acc")
xlabel("$t [hour]$",'Interpreter','latex')
ylabel("heading  $[rad]$",'Interpreter','latex')
legend(Location="northwest")
title('Plot estimated heading against true heading')
grid on
%%
turn_rate_raw=diff(resample_dataset_clean.heading);
figure
hold on
plot(Tsec(2:end)./(3600),turn_rate_raw,"b--",'LineWidth',4,DisplayName="turn_rate_raw")
plot(Tsec(2:end)/(3600),estimateStates(5,2:end),"r",'LineWidth',4,DisplayName="turn_rate")
xlabel("$t [hour]$",'Interpreter','latex')
ylabel("heading  $[rad]$",'Interpreter','latex')
legend(Location="northwest")
title('Plot estimated heading against true heading')
grid on