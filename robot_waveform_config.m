function config = robot_waveform_config()
%ROBOT_WAVEFORM_CONFIG Shared HFM-synchronized FSK settings for both endpoints.
config.SampleRate = 48000;
config.BitRate = 500;
config.SamplesPerBit = config.SampleRate / config.BitRate;
config.CarrierZero = 17000;
config.CarrierOne = 17800;
config.HfmDuration = 0.025;
config.HfmStartFrequency = 14500;
config.HfmEndFrequency = 19500;
config.HfmSearchConfidence = 0.2;
config.HfmMinConfidence = 0.4;
config.GuardDuration = 0.005;
config.FrameBytes = 38;
config.PayloadBits = config.FrameBytes * 8;
config.HfmSamples = round(config.SampleRate * config.HfmDuration);
config.GuardSamples = round(config.SampleRate * config.GuardDuration);
config.PayloadSamples = config.PayloadBits * config.SamplesPerBit;
config.MinimumDecodeSamples = config.HfmSamples + config.GuardSamples + ...
    config.PayloadSamples;
config.WaveformSamples = config.MinimumDecodeSamples + config.GuardSamples;
config.CaptureRestartSamples = 30 * config.SampleRate;
end
