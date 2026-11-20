function compensation = furuta_pwm_compensation_config(fit,referenceVoltage,sampleTime)
% Use the measured conducting knots and the common equivalent-voltage scale.
voltage = fit.voltageKnots(:).';
current = fit.currentKnots(:).';
conducting = current>0;
voltage = voltage(conducting);
current = current(conducting);
currentPerVolt = interp1(voltage,current,referenceVoltage,'pchip')/referenceVoltage;
compensation = struct('voltage',voltage,'current',current, ...
    'currentPerVolt',currentPerVolt,'sampleTime',sampleTime, ...
    'equivalentVoltageLimit',current(end)/currentPerVolt);
end
