function config = robot_waveform_config()
%ROBOT_WAVEFORM_CONFIG Shared fixed-rate modem settings for master and slave.
config.SampleRate = 48000;
config.BitRate = 1000;
config.SamplesPerBit = config.SampleRate / config.BitRate;
config.CarrierZero = 17000;
config.CarrierOne = 17800;
config.HfmDuration = 0.025;
config.HfmStartFrequency = 14500;
config.HfmEndFrequency = 19500;
config.GuardDuration = 0.005;
config.FrameBytes = 38;
config.PayloadBits = config.FrameBytes * 8;
config.HfmSamples = round(config.SampleRate * config.HfmDuration);
config.GuardSamples = round(config.SampleRate * config.GuardDuration);
config.PayloadSamples = config.PayloadBits * config.SamplesPerBit;
config.WaveformSamples = config.HfmSamples + config.GuardSamples + ...
    config.PayloadSamples;
end
