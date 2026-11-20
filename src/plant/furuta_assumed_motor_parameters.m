function [drive,p] = furuta_assumed_motor_parameters(p)
% Provisional GA25-370 parameters from supplied assumptions and synthetic fits.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
results = fullfile(root,'parameter estimation','example fit results');
motor = readtable(fullfile(results,'synthetic_motor_fit.csv'));
mechanical = readtable(fullfile(results,'synthetic_friction_gearbox_fit.csv'));

drive.voltageLimit = 12;
drive.resistance = motor.R_ohm;
drive.backEmfOutput = motor.Ke_V_per_rad_s;
drive.torqueConstantOutput = 0.15;
drive.gearRatioSigned = -35.0048;
% The model orients the motor-positive coordinate to agree with the arm.
drive.gearRatio = abs(drive.gearRatioSigned);
drive.motorShaftDamping = 4.08e-9; % N*m*s/rad, motor shaft
drive.armatureInductance = 12.003374e-6; % H
drive.gearEfficiency = mechanical.gear_efficiency_fraction;
drive.speedGain = motor.speedGain_rad_s_per_V;
drive.timeConstant = motor.timeConstant_s;

% Reflect the motor-shaft friction at the output. Retain the previous
% synthetic output-inertia estimate until a replacement is available.
drive.viscousDamping = drive.motorShaftDamping*drive.gearRatio^2;
fitDamping = drive.torqueConstantOutput/(drive.resistance*drive.speedGain);
drive.reflectedInertia = drive.timeConstant*fitDamping;

% Equivalent conversion for the explicit gearbox gains in the block model.
drive.backEmfMotor = drive.backEmfOutput/drive.gearRatio;
drive.torqueConstantMotor = drive.torqueConstantOutput/(drive.gearRatio*drive.gearEfficiency);
p.armInertia = p.armInertia+drive.reflectedInertia;
p.armDamping = mechanical.arm_damping_Nm_s_per_rad;
p.pendulumDamping = mechanical.hinge_damping_Nm_s_per_rad;
end
