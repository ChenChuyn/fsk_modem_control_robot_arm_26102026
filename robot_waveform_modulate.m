function samples = robot_waveform_modulate(frame)
%ROBOT_WAVEFORM_MODULATE HFM synchronization followed by a binary FSK packet.
config = robot_waveform_config();
if ~isa(frame, 'uint8') || numel(frame) ~= config.FrameBytes
    error('robot_waveform_modulate:InvalidFrame', ...
        'Input must be a %d-byte uint8 frame.', config.FrameBytes);
end

frame = reshape(frame, 1, []);
bitPositions = repmat(8:-1:1, numel(frame), 1);
byteValues = repmat(frame(:), 1, 8);
bits = reshape(bitget(byteValues, bitPositions).', 1, []);
bitMask = repelem(bits, config.SamplesPerBit);
sampleTime = (0:numel(bitMask) - 1) / config.SampleRate;
payload = 0.8 * ((bitMask == 0) .* cos(2*pi*config.CarrierZero*sampleTime) + ...
    (bitMask == 1) .* cos(2*pi*config.CarrierOne*sampleTime));
samples = [robot_hfm_template(config), zeros(1, config.GuardSamples), ...
    payload, zeros(1, config.GuardSamples)];
end
