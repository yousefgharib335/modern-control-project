function result = fit_example_motor
% Fit and validate an effective DC motor model on the synthetic practice log.
root = fileparts(mfilename('fullpath'));
data = readtable(fullfile(root,'example_motor_readings_synthetic.csv'));
t = data.time_s;
voltage = data.voltage_V;
measuredRPM = data.speed_rpm;
measuredCurrent = data.current_A;
assert(all(isfinite([t voltage measuredRPM measuredCurrent]),'all') && all(diff(t)>0), ...
    'The motor log must contain finite readings and strictly increasing times.');

% The final +6 V pulse and its decay are held out. Predict a 0.5 s lead-in
% from rest to initialize the simulation before the held-out pulse.
fitRows = t < 17.5;
checkRows = t >= 19;
checkRun = t >= 18.5;
checkInRun = t(checkRun) >= 19;
fitInput = voltage(fitRows);
fitTime = t(fitRows);
fitRPM = measuredRPM(fitRows);
fitCurrent = measuredCurrent(fitRows);

% Parameters: R in ohms, Ke in V/(rad/s), DC speed gain in (rad/s)/V,
% and mechanical response time constant in seconds. Optimize log values
% so every coefficient remains positive.
initial = [8 0.4 1.5 0.35];
speedNoiseRPM = 0.65;
currentNoiseA = 0.008;
objective = @(logValues) weightedError(logValues,fitTime,fitInput, ...
    fitRPM,fitCurrent,speedNoiseRPM,currentNoiseA);
options = optimset('Display','off','TolX',1e-9,'TolFun',1e-9, ...
    'MaxIter',1500,'MaxFunEvals',3000);
[fit,~,exitFlag] = fminsearch(objective,log(initial),options);
assert(exitFlag > 0,'Optimization did not converge.');
values = exp(fit);
[trainingRPM,trainingCurrent] = predictMotor(values,fitTime,fitInput);
[testRPM,testCurrent] = predictMotor(values,t(checkRun),voltage(checkRun));
testRPM = testRPM(checkInRun);
testCurrent = testCurrent(checkInRun);
trainingRMSE = [rmsError(trainingRPM,fitRPM),rmsError(trainingCurrent,fitCurrent)];
testRMSE = [rmsError(testRPM,measuredRPM(checkRows)), ...
    rmsError(testCurrent,measuredCurrent(checkRows))];
assert(all(isfinite(testRMSE)),'Held-out predictions must be finite.');
assert(testRMSE(1)<2 && testRMSE(2)<0.025, ...
    'Held-out predictions deviate more than expected for the synthetic noise.');

result = table(values(1),values(2),values(3),values(4), ...
    12*values(3)*60/(2*pi),trainingRMSE(1),trainingRMSE(2), ...
    testRMSE(1),testRMSE(2),nnz(fitRows),nnz(checkRows),'VariableNames', ...
    {'R_ohm','Ke_V_per_rad_s','speedGain_rad_s_per_V','timeConstant_s', ...
    'noLoadRPM_at_12V','fit_RMSE_rpm','fit_RMSE_A','validation_RMSE_rpm', ...
    'validation_RMSE_A','fit_samples','validation_samples'});
validation = table(t(checkRows),voltage(checkRows),measuredRPM(checkRows),testRPM, ...
    measuredCurrent(checkRows),testCurrent,'VariableNames', ...
    {'time_s','voltage_V','measured_speed_rpm','predicted_speed_rpm', ...
    'measured_current_A','predicted_current_A'});
output = fullfile(root,'example fit results');
if ~isfolder(output)
    mkdir(output);
end
writetable(result,fullfile(output,'synthetic_motor_fit.csv'));
writetable(validation,fullfile(output,'synthetic_motor_validation.csv'));
disp(result);
end

function cost = weightedError(logValues,t,voltage,speedRPM,current,speedScale,currentScale)
[rpm,amps] = predictMotor(exp(logValues),t,voltage);
cost = mean(((rpm-speedRPM)/speedScale).^2 + ((amps-current)/currentScale).^2);
if ~isfinite(cost)
    cost = realmax;
end
end

function [rpm,current] = predictMotor(values,t,voltage)
resistance = values(1);
backEMF = values(2);
speedGain = values(3);
timeConstant = values(4);
omega = zeros(size(t));
for k = 2:numel(t)
    target = speedGain*voltage(k-1);
    fraction = -expm1(-(t(k)-t(k-1))/timeConstant);
    omega(k) = omega(k-1) + fraction*(target-omega(k-1));
end
rpm = omega*60/(2*pi);
current = (voltage-backEMF*omega)/resistance;
end

function value = rmsError(prediction,measured)
value = sqrt(mean((prediction-measured).^2));
end
