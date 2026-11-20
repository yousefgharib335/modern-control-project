function result = fit_example_friction_gearbox
% Generate and fit synthetic friction and gearbox experiments for practice.
folder = fileparts(mfilename('fullpath'));
addpath(fullfile(fileparts(folder),'src','plant'));
p = furuta_cad_parameters;
output = fullfile(folder,'example fit results');
if ~isfolder(output)
    mkdir(output);
end
previousRng = rng;
restoreRng = onCleanup(@()rng(previousRng)); %#ok<NASGU>
rng(2519,'twister');
t = (0:0.01:4).';

% CAD inertia seen by the arm when the pendulum is fixed at hanging.
hangRotation = diag([1 -1 -1]);
position = p.hingeTranslation + hangRotation*p.pendulumCOMArmZero;
pendulumInertia = hangRotation*p.pendulumInertiaArmZero*hangRotation.';
armInertia = p.armInertia + p.pendulumMass*sum(position(1:2).^2) + pendulumInertia(3,3);

% Arm fixed during the pendulum test. Angle is measured from hanging.
comRadius = norm(p.pendulumCOMArmZero(2:3));
hingeInertia = p.pendulumInertiaArmZero(1,1) + p.pendulumMass*comRadius^2;
gravityMoment = p.pendulumMass*p.gravity*comRadius;

% Practice values used to generate the observations only.
trueArm = 1.1e-4;
trueHinge = 2.4e-5;
trueRatio = 30;
trueEfficiency = 0.75;
angleNoise = deg2rad(0.08);
arm = @(b,rate) armAngle(t,b,armInertia,rate);
hinge = @(b,angle) hingeAngle(t,b,hingeInertia,gravityMoment,angle);

armTrain = arm(trueArm,2.5) + angleNoise*randn(size(t));
armCheck = arm(trueArm,-1.5) + angleNoise*randn(size(t));
hingeTrain = hinge(trueHinge,deg2rad(12)) + angleNoise*randn(size(t));
hingeCheck = hinge(trueHinge,deg2rad(-8)) + angleNoise*randn(size(t));
armDamping = fitDamping(@(b)arm(b,2.5),armTrain);
hingeDamping = fitDamping(@(b)hinge(b,deg2rad(12)),hingeTrain);
armPrediction = arm(armDamping,-1.5);
hingePrediction = hinge(hingeDamping,deg2rad(-8));
armError = rad2deg(sqrt(mean((armPrediction-armCheck).^2)));
hingeError = rad2deg(sqrt(mean((hingePrediction-hingeCheck).^2)));

% Simulated steady loaded measurements on both shafts, one drive direction.
% Estimate speed ratio and mechanical power efficiency from separate channels.
outputRPM = [40;65;85;105;125;155;70;115;145];
inputTorque = [0.0028;0.0065;0.0091;0.0043;0.0062;0.0034;0.0048;0.008;0.0037];
inputRPM = trueRatio*outputRPM + 2.5*randn(size(outputRPM));
outputRPM = outputRPM + 0.12*randn(size(outputRPM));
outputTorque = trueRatio*trueEfficiency*inputTorque + 0.001*randn(size(inputTorque));
inputTorque = inputTorque + 0.00004*randn(size(inputTorque));
trainGear = (1:6).';
checkGear = (7:9).';
ratio = outputRPM(trainGear)\inputRPM(trainGear);
inputPower = inputTorque.*inputRPM*2*pi/60;
outputPower = outputTorque.*outputRPM*2*pi/60;
efficiency = inputPower(trainGear)\outputPower(trainGear);
gearRPMError = sqrt(mean((inputRPM(checkGear)/ratio-outputRPM(checkGear)).^2));
gearPowerError = sqrt(mean((efficiency*inputPower(checkGear)-outputPower(checkGear)).^2));

assert(armError<0.15 && hingeError<0.15 && gearRPMError<0.5 && gearPowerError<0.07, ...
    'The held-out predictions exceed expected synthetic measurement noise.');
split = [repmat("fit",6,1);repmat("validation",3,1)];
gear = table(split,inputRPM,outputRPM,inputTorque,outputTorque,inputPower,outputPower, ...
    inputRPM/ratio,efficiency*inputPower,'VariableNames',{'split','motor_speed_rpm', ...
    'output_speed_rpm','motor_torque_Nm','output_torque_Nm','motor_power_W','output_power_W', ...
    'predicted_output_speed_rpm','predicted_output_power_W'});
angles = table(t,rad2deg(armTrain),rad2deg(armCheck),rad2deg(armPrediction), ...
    rad2deg(hingeTrain),rad2deg(hingeCheck),rad2deg(hingePrediction),'VariableNames', ...
    {'time_s','arm_fit_deg','arm_validation_deg','arm_predicted_deg', ...
    'hinge_fit_deg','hinge_validation_deg','hinge_predicted_deg'});

result = table(armDamping,hingeDamping,ratio,efficiency,armError,hingeError, ...
    gearRPMError,gearPowerError,'VariableNames',{'arm_damping_Nm_s_per_rad', ...
    'hinge_damping_Nm_s_per_rad','gear_ratio_motor_to_output','gear_efficiency_fraction', ...
    'arm_validation_RMSE_deg','hinge_validation_RMSE_deg','gear_validation_RMSE_rpm', ...
    'gear_validation_power_RMSE_W'});
writetable(result,fullfile(output,'synthetic_friction_gearbox_fit.csv'));
writetable(angles,fullfile(output,'synthetic_friction_trials.csv'));
writetable(gear,fullfile(output,'synthetic_gearbox_trials.csv'));
disp(result);
end

function angle = armAngle(t,damping,inertia,initialRate)
rateDecay = damping/inertia;
angle = initialRate * (-expm1(-rateDecay*t)) / rateDecay;
end

function angle = hingeAngle(t,damping,inertia,gravityMoment,initialAngle)
options = odeset('RelTol',1e-9,'AbsTol',1e-11);
[~,x] = ode45(@(~,q)[q(2);(-damping*q(2)-gravityMoment*sin(q(1)))/inertia], ...
    t,[initialAngle;0],options);
angle = x(:,1);
end

function damping = fitDamping(predict,observed)
options = optimset('Display','off','TolX',1e-9,'TolFun',1e-12);
cost = @(x) mean((predict(exp(x))-observed).^2);
[fit,~,flag] = fminsearch(cost,log(5e-5),options);
assert(flag>0,'Damping estimation did not converge.');
damping = exp(fit);
end
