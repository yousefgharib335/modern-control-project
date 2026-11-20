function [A,B,C,D] = furuta_voltage_linearize(p,drive)
% Upright small-signal model with input voltage and output [angles; rates].
x = zeros(4,1);
torqueGain = drive.torqueConstantOutput/drive.resistance;
speedLoss = torqueGain*drive.backEmfOutput+drive.viscousDamping;
torqueInput = furuta_cad_dynamics(x,1,p)-furuta_cad_dynamics(x,0,p);
A = zeros(4);
step = 1e-20;
for k = 1:4
    perturbation = x;
    perturbation(k) = 1i*step;
    A(:,k) = imag(furuta_cad_dynamics(perturbation,-speedLoss*perturbation(3),p))/step;
end
B = torqueInput*torqueGain;
C = eye(4);
D = zeros(4,1);
end
