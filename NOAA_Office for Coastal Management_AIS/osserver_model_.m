
dt = 180;     % seconds
simTime = 18000; % seconds
tspan = 0:dt:simTime;
trueInitialState = [2.5e05; 2.7e06; 17.9; 4.31;0;0]; 
initialCovariance = 0.0001*eye(6);
processNoise = diag([5; 5; 0.5; 0.1;1;1]); % Process noise matrix
measureNoise = diag([1e-4;1e-4;1e-4;1e-4]); % Measurement noise matrix. Units are m^2 and rad^2.
%%
numSteps = length(tspan);
trueStates = NaN(6,numSteps);
trueStates(:,1) = trueInitialState;
estimateStates = NaN(size(trueStates));
measurements = NaN(4,numSteps);
%%
for i = 2:length(tspan)
    if i ~= 1
        trueStates(:,i) = stateModel(trueStates(:,i-1),dt) + sqrt(processNoise)*randn(6,1);  
    end
    measurements(:,i) = measureModel(trueStates(:,i)) + sqrt(measureNoise)*randn(4,1);
end


%%
figure(1)
plot(trueStates(1,1),trueStates(2,1),"r*",DisplayName="Initial Truth")
hold on
plot(trueStates(1,:),trueStates(2,:),"r",DisplayName="True Trajectory")
xlabel("x (m)")
ylabel("y (m)")
title("True Trajectory")
axis square
%%
obj = extendedKalmanFilter('HasAdditiveMeasurementNoise',true);
obj.StateTransitionFcn = @(x,u)stateModel(x,dt);
obj.MeasurementFcn =  @(x,u)measureModel(x);
obj.StateCovariance=initialCovariance;
obj.ProcessNoise=processNoise;
obj.MeasurementNoise=measureNoise;
obj.State = trueInitialState;

%%
for i=2:length(tspan)
    correct(obj,measurements(:,i),[])
    predict(obj,measurements(:,i))
    estimateStates(:,i)=obj.State;
end
%%
figure(1)
hold on
plot(trueStates(5,:),trueStates(6,:),"r",DisplayName="True Trajectory")
plot(estimateStates(5,:),estimateStates(6,:),"b--",DisplayName="Estiamate")
legend(Location="northwest")

xlabel("x (m)")
ylabel("y (m)")
title("True Trajectory")
axis square
