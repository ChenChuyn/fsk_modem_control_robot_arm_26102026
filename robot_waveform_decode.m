function [frame, valid, reason, syncConfidence, startIndex] = robot_waveform_decode(samples)
%ROBOT_WAVEFORM_DECODE HFM-correlate, FSK-demodulate, and validate a frame.
config = robot_waveform_config();
frame = zeros(1, config.FrameBytes, 'uint8');
valid = false;
reason = 'Insufficient samples';
syncConfidence = 0;
startIndex = 1;
samples = double(samples(:).');

if numel(samples) < config.MinimumDecodeSamples
    return;
end

[~, analyticTemplate] = robot_hfm_template(config);
searchLength = numel(samples) - config.HfmSamples + 1;
searchSamples = samples(1:searchLength + config.HfmSamples - 1);
fftLength = 2^nextpow2(numel(searchSamples) + config.HfmSamples - 1);
correlation = ifft(fft(searchSamples, fftLength) .* ...
    fft(conj(fliplr(analyticTemplate)), fftLength));
correlation = correlation(config.HfmSamples: ...
    searchLength + config.HfmSamples - 1);

energyPrefix = [0, cumsum(samples(1:searchLength + config.HfmSamples - 1).^2)];
windowEnergy = energyPrefix(config.HfmSamples + (1:searchLength)) - ...
    energyPrefix(1:searchLength);
correlationScale = sqrt(windowEnergy * sum(abs(analyticTemplate).^2));
confidence = zeros(size(correlation));
activeWindows = windowEnergy > eps;
confidence(activeWindows) = abs(correlation(activeWindows)) ./ ...
    correlationScale(activeWindows);
[syncConfidence, startIndex] = max(confidence);
if syncConfidence < config.HfmSearchConfidence
    reason = 'HFM synchronization not found';
    return;
end

if syncConfidence >= config.HfmMinConfidence
    [frame, valid, reason] = decodeFskPayload(samples, startIndex, 1, config);
    if valid || strcmp(reason, 'Incomplete FSK payload')
        return;
    end
end

[syncConfidence, startIndex, sampleRateRatio] = refineHfmTiming( ...
    samples, startIndex, syncConfidence, analyticTemplate, config);
if syncConfidence < config.HfmMinConfidence
    reason = 'HFM synchronization not found';
    return;
end
[frame, valid, reason] = decodeFskPayload( ...
    samples, startIndex, sampleRateRatio, config);
end

function [frame, valid, reason] = decodeFskPayload( ...
        samples, startIndex, sampleRateRatio, config)
frame = zeros(1, config.FrameBytes, 'uint8');
valid = false;
payloadIndices = startIndex + sampleRateRatio*(config.HfmSamples + ...
    config.GuardSamples + (0:config.PayloadSamples - 1));
if payloadIndices(end) > numel(samples)
    reason = 'Incomplete FSK payload';
    return;
end

payload = interp1(1:numel(samples), samples, payloadIndices, 'linear');
[energyZero, energyOne] = toneEnergies(payload, config);
payloadEnergyZero = energyZero(config.SamplesPerBit:config.SamplesPerBit:end);
payloadEnergyOne = energyOne(config.SamplesPerBit:config.SamplesPerBit:end);
bits = payloadEnergyOne > payloadEnergyZero;
bytes = reshape(bits, 8, []).';
frame = uint8(bi2de(bytes, 'left-msb').');

[~, valid, ~, reason] = robot_packet_decode(frame);
end

function [bestConfidence, bestStart, bestRatio] = refineHfmTiming( ...
        samples, initialStart, initialConfidence, template, config)
ratioCandidates = 0.995:0.0001:1.005;
startRadius = 64;
bestConfidence = initialConfidence;
bestStart = initialStart;
bestRatio = 1;

for ratio = ratioCandidates
    templateIndices = 1:1/ratio:config.HfmSamples;
    scaledTemplate = interp1(1:config.HfmSamples, template, ...
        templateIndices, 'linear');
    firstStart = max(1, initialStart - startRadius);
    lastStart = min(initialStart + startRadius, ...
        numel(samples) - numel(scaledTemplate) + 1);
    if lastStart < firstStart
        continue;
    end

    segment = samples(firstStart:lastStart + numel(scaledTemplate) - 1);
    correlation = conv(segment, conj(fliplr(scaledTemplate)), 'valid');
    energyPrefix = [0, cumsum(segment.^2)];
    windowEnergy = energyPrefix(numel(scaledTemplate) + ...
        (1:numel(correlation))) - energyPrefix(1:numel(correlation));
    confidence = abs(correlation) ./ max(sqrt(windowEnergy * ...
        sum(abs(scaledTemplate).^2)), eps);
    [candidateConfidence, candidateIndex] = max(confidence);
    if candidateConfidence > bestConfidence
        bestConfidence = candidateConfidence;
        bestStart = firstStart + candidateIndex - 1;
        bestRatio = ratio;
    end
end
end

function [energyZero, energyOne] = toneEnergies(samples, config)
sampleTime = (0:config.SamplesPerBit - 1)/config.SampleRate;
cosZero = cos(2*pi*config.CarrierZero*sampleTime);
sinZero = sin(2*pi*config.CarrierZero*sampleTime);
cosOne = cos(2*pi*config.CarrierOne*sampleTime);
sinOne = sin(2*pi*config.CarrierOne*sampleTime);

zeroI = filter(fliplr(cosZero), 1, samples);
zeroQ = filter(fliplr(sinZero), 1, samples);
oneI = filter(fliplr(cosOne), 1, samples);
oneQ = filter(fliplr(sinOne), 1, samples);
energyZero = zeroI.^2 + zeroQ.^2;
energyOne = oneI.^2 + oneQ.^2;
end
