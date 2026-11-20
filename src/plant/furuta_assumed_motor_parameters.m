function [drive,p] = furuta_assumed_motor_parameters(p)
% Output-side GA25-370 practice parameters from the synthetic fits.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
results = fullfile(root,'parameter estimation','example fit results');
motor = readtable(fullfile(results,'synthetic_motor_fit.csv'));
mechanical = readtable(fullfile(results,'synthetic_friction_gearbox_fit.csv'));

drive.voltageLimit = 12;
drive.resistance = motor.R_ohm;
drive.backEmfOutput = motor.Ke_V_per_rad_s;
drive.torqueConstantOutput = 0.15;
drive.gearRatio = mechanical.gear_ratio_motor_to_output;
drive.gearEfficiency = mechanical.gear_efficiency_fraction;
drive.speedGain = motor.speedGain_rad_s_per_V;
drive.timeConstant = motor.timeConstant_s;

% The unloaded speed fit implies both the total speed-feedback coefficient
% and reflected inertia once the effective torque constant is assumed.
totalDamping = drive.torqueConstantOutput/(drive.resistance*drive.speedGain);
electricalDamping = drive.torqueConstantOutput*drive.backEmfOutput/drive.resistance;
drive.viscousDamping = totalDamping-electricalDamping;
drive.reflectedInertia = drive.timeConstant*totalDamping;
assert(drive.viscousDamping >= 0,'FurutaMotor:Fit','The practice fit implies negative drive friction.');

% Equivalent conversion for the explicit gearbox gains in the block model.
drive.backEmfMotor = drive.backEmfOutput/drive.gearRatio;
drive.torqueConstantMotor = drive.torqueConstantOutput/(drive.gearRatio*drive.gearEfficiency);
p.armInertia = p.armInertia+drive.reflectedInertia;
p.armDamping = mechanical.arm_damping_Nm_s_per_rad;
p.pendulumDamping = mechanical.hinge_damping_Nm_s_per_rad;
end
