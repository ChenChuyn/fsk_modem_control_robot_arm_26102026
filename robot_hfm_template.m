function [template, analyticTemplate] = robot_hfm_template(config)
%ROBOT_HFM_TEMPLATE Generate the HFM sync pulse and phase-independent template.
t = (0:config.HfmSamples - 1) / config.SampleRate;
f0 = config.HfmStartFrequency;
f1 = config.HfmEndFrequency;
duration = config.HfmDuration;
sweep = f1 - f0;
phaseScale = f0*f1*duration / sweep;
phase = 2*pi*phaseScale .* log((f1*duration) ./ ...
    (f1*duration - sweep*t));
template = 0.8*cos(phase);
analyticTemplate = exp(1i*phase);
end
