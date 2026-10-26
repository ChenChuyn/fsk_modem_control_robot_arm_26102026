function samples = robot_waveform_modulate(frame)
%ROBOT_WAVEFORM_MODULATE HFM synchronization followed by fixed-length binary FSK.
config = robot_waveform_config();
if ~isa(frame, 'uint8') || numel(frame) ~= config.FrameBytes
    error('robot_waveform_modulate:InvalidFrame', ...
        'Input must be a %d-byte uint8 frame.', config.FrameBytes);
end

frame = reshape(frame, 1, []);
bitPositions = repmat(8:-1:1, numel(frame), 1);
byteValues = repmat(frame(:), 1, 8);
bits = reshape(bitget(byteValues, bitPositions).', 1, []);
sampleTime = (0:config.SamplesPerBit - 1) / config.SampleRate;
payload = zeros(1, config.PayloadSamples);

for bitIndex = 1:config.PayloadBits
    if bits(bitIndex)
        frequency = config.CarrierOne;
    else
        frequency = config.CarrierZero;
    end
    sampleIndices = (bitIndex - 1)*config.SamplesPerBit + (1:config.SamplesPerBit);
    payload(sampleIndices) = 0.8*cos(2*pi*frequency*sampleTime);
end

samples = [robot_hfm_template(config), zeros(1, config.GuardSamples), payload];
end
