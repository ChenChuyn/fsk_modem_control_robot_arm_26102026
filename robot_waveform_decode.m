function [frame, valid, reason, syncConfidence, startIndex] = robot_waveform_decode(samples)
%ROBOT_WAVEFORM_DECODE HFM-correlate, FSK-demodulate, and validate one fixed frame.
config = robot_waveform_config();
frame = zeros(1, config.FrameBytes, 'uint8');
valid = false;
reason = 'Insufficient samples';
syncConfidence = 0;
startIndex = 1;
samples = double(samples(:).');

if numel(samples) < config.WaveformSamples
    return;
end

template = robot_hfm_template(config);
searchLength = numel(samples) - config.WaveformSamples + 1;
searchSamples = samples(1:searchLength + config.HfmSamples - 1);
fftLength = 2^nextpow2(numel(searchSamples) + config.HfmSamples - 1);
correlation = ifft(fft(searchSamples, fftLength) .* ...
    fft(conj(fliplr(template)), fftLength));
correlation = correlation(config.HfmSamples:searchLength + config.HfmSamples - 1);

energyPrefix = [0, cumsum(searchSamples.^2)];
windowEnergy = energyPrefix(config.HfmSamples + (1:searchLength)) - ...
    energyPrefix(1:searchLength);
confidence = abs(correlation) ./ ...
    max(sqrt(windowEnergy * sum(abs(template).^2)), eps);
[syncConfidence, startIndex] = max(confidence);
if syncConfidence < 0.2
    reason = 'HFM synchronization not found';
    return;
end

payloadStart = startIndex + config.HfmSamples + config.GuardSamples;
payloadEnd = payloadStart + config.PayloadSamples - 1;
if payloadEnd > numel(samples)
    reason = 'Incomplete FSK payload';
    return;
end

payload = samples(payloadStart:payloadEnd);
symbolMatrix = reshape(payload, config.SamplesPerBit, []);
t = (0:config.SamplesPerBit - 1).' / config.SampleRate;
cosZero = cos(2*pi*config.CarrierZero*t);
sinZero = sin(2*pi*config.CarrierZero*t);
cosOne = cos(2*pi*config.CarrierOne*t);
sinOne = sin(2*pi*config.CarrierOne*t);
energyZero = (cosZero.'*symbolMatrix).^2 + (sinZero.'*symbolMatrix).^2;
energyOne = (cosOne.'*symbolMatrix).^2 + (sinOne.'*symbolMatrix).^2;
bits = energyOne > energyZero;
bytes = reshape(bits, 8, []).';
frame = uint8(bi2de(bytes, 'left-msb').');

[~, valid, ~, reason] = robot_packet_decode(frame);
end
