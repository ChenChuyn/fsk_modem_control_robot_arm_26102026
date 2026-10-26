function tests = test_robot_packet
tests = functiontests(localfunctions);
end

function testRoundTripPreservesPoseAndSequence(testCase)
angles = [0, 90, 135, 180];
gyro = [-12.34, 0, 327.67];
frame = robot_packet_encode(angles, gyro, 65535, 1234);
[packet, valid, corrected, reason] = robot_packet_decode(frame);

verifyTrue(testCase, valid, reason);
verifyEqual(testCase, numel(frame), 38);
verifyEqual(testCase, packet.angles, angles);
verifyEqual(testCase, packet.gyro, gyro);
verifyEqual(testCase, packet.session, 1234);
verifyEqual(testCase, packet.sequence, 65535);
verifyEqual(testCase, corrected, 0);
end

function testSecdedCorrectsEverySingleCodeBit(testCase)
angles = [24, 71, 119, 160];
gyro = [1.25, -9.5, 22.75];
frame = robot_packet_encode(angles, gyro, 42, 52);

for codeword = 1:17
    for codeBit = 1:13
        corrupted = frame;
        highByte = 5 + 2*(codeword - 1);
        if codeBit <= 8
            corrupted(highByte) = bitxor(corrupted(highByte), ...
                bitshift(uint8(1), 8 - codeBit));
        else
            corrupted(highByte + 1) = bitxor(corrupted(highByte + 1), ...
                bitshift(uint8(1), 16 - codeBit));
        end

        [packet, valid, corrected, reason] = robot_packet_decode(corrupted);
        verifyTrue(testCase, valid, reason);
        verifyEqual(testCase, packet.angles, angles);
        verifyEqual(testCase, packet.gyro, gyro);
        verifyEqual(testCase, corrected, 1);
    end
end
end

function testSecdedRejectsDoubleBitError(testCase)
frame = robot_packet_encode([30, 60, 120, 150], [0, 0, 0], 10, 9);
frame(6) = bitxor(frame(6), uint8(192));
[~, valid, ~, reason] = robot_packet_decode(frame);

verifyFalse(testCase, valid);
verifyEqual(testCase, reason, 'Uncorrectable SECDED error');
end

function testSecdedRejectsOutOfRangeSyndromeWithoutThrowing(testCase)
frame = robot_packet_encode([30, 60, 120, 150], [0, 0, 0], 10, 9);
frame(5) = bitxor(frame(5), uint8(hex2dec('91')));
[~, valid, ~, reason] = robot_packet_decode(frame);

verifyFalse(testCase, valid);
verifyEqual(testCase, reason, 'Uncorrectable SECDED error');
end

function testLowAmplitudeSignalWithNoise(testCase)
frame = robot_packet_encode([5, 60, 130, 175], [0.5, -1.25, 5], 12, 18);
signal = robot_waveform_modulate(frame);
rng(28);
received = 0.1*signal + 0.08*randn(size(signal));
[decodedFrame, valid, reason] = robot_waveform_decode(received);

verifyTrue(testCase, valid, reason);
verifyEqual(testCase, decodedFrame, frame);
end

function testAmbientNoiseDoesNotDecodeAsFrame(testCase)
config = robot_waveform_config();
rng(31);
noise = 0.05*randn(1, config.MinimumDecodeSamples);
[~, valid, reason, confidence] = robot_waveform_decode(noise);

verifyFalse(testCase, valid);
verifyEqual(testCase, reason, 'HFM synchronization not found');
verifyLessThan(testCase, confidence, config.HfmMinConfidence);
end

function testVeryWeakSignalIsNotAcceptedAsCorruptPacket(testCase)
config = robot_waveform_config();
frame = robot_packet_encode([22, 79, 131, 168], ...
    [-77.25, 0.5, 120.75], 456, 789);
signal = robot_waveform_modulate(frame);
rng(42);
for amplitude = [0.8, 0.3, 0.1]
    for noiseLevel = [0, 0.03, 0.08, 0.15]
        received = amplitude*signal + noiseLevel*randn(size(signal));
    end
end
[~, valid, reason, confidence] = robot_waveform_decode(received);

verifyFalse(testCase, valid);
verifyEqual(testCase, reason, 'HFM synchronization not found');
verifyLessThan(testCase, confidence, config.HfmMinConfidence);
end

function testHfmFskWaveformRoundTripWithOffsetAndNoise(testCase)
angles = [20, 80, 130, 175];
gyro = [-123.45, 3.21, 88.88];
frame = robot_packet_encode(angles, gyro, 200, 201);
config = robot_waveform_config();
signal = robot_waveform_modulate(frame);
rng(7);
prefix = zeros(1, 23);
suffix = zeros(1, config.WaveformSamples);
noisySignal = [prefix, signal, suffix] + 0.015*randn( ...
    1, numel(prefix) + numel(signal) + numel(suffix));
[decodedFrame, valid, reason, confidence, startIndex] = ...
    robot_waveform_decode(noisySignal);

verifyTrue(testCase, valid, reason);
verifyEqual(testCase, decodedFrame, frame);
verifyGreaterThan(testCase, confidence, 0.2);
verifyEqual(testCase, startIndex, numel(prefix) + 1);
end

function testHfmFskWaveformHandlesGainAndEcho(testCase)
frame = robot_packet_encode([10, 45, 130, 175], [-4.5, 0, 20.25], 7, 13);
signal = robot_waveform_modulate(frame);
rng(19);
echo = [zeros(1, 19), 0.35*signal(1:end - 19)];
received = 0.25*signal + echo + 0.003*randn(size(signal));
[decodedFrame, valid, reason] = robot_waveform_decode(received);

verifyTrue(testCase, valid, reason);
verifyEqual(testCase, decodedFrame, frame);
end

function testHfmFskDecodesWithoutTrailingGuard(testCase)
frame = robot_packet_encode([45, 80, 110, 160], [1.25, -2.5, 5], 8, 14);
config = robot_waveform_config();
signal = robot_waveform_modulate(frame);
received = signal(1:config.MinimumDecodeSamples);
[decodedFrame, valid, reason] = robot_waveform_decode(received);

verifyTrue(testCase, valid, reason);
verifyEqual(testCase, decodedFrame, frame);
end

function testHfmFskCanResumeAnIncompleteCapture(testCase)
frame = robot_packet_encode([35, 70, 125, 155], [3.5, -7, 12], 9, 15);
config = robot_waveform_config();
signal = robot_waveform_modulate(frame);
prefixLength = 20000;
partialLength = config.MinimumDecodeSamples - 3000;
buffer = SampleRingBuffer(config.WaveformSamples + prefixLength);
buffer.push([zeros(1, prefixLength), signal(1:partialLength)]);
[~, valid, reason, ~, syncStart] = ...
    robot_waveform_decode(buffer.peek(buffer.Count));

verifyFalse(testCase, valid);
verifyEqual(testCase, reason, 'Incomplete FSK payload');
buffer.discard(syncStart - 1);
buffer.push(signal(partialLength + 1:config.MinimumDecodeSamples));
[decodedFrame, valid, reason] = robot_waveform_decode(buffer.peek(buffer.Count));

verifyTrue(testCase, valid, reason);
verifyEqual(testCase, decodedFrame, frame);
end

function testHfmFskToleratesIndependentAudioClocks(testCase)
frame = robot_packet_encode([15, 75, 120, 170], [-7.5, 0.25, 99.99], 23, 44);
config = robot_waveform_config();
signal = robot_waveform_modulate(frame);

for clockRatio = [0.995, 0.9999, 1.0001, 1.005]
    txTime = (0:numel(signal) - 1)/config.SampleRate;
    rxTime = (0:ceil(numel(signal)*clockRatio) - 1)/config.SampleRate;
    warped = interp1(txTime, signal, rxTime/clockRatio, 'linear', 0);
    received = [zeros(1, 17), warped, zeros(1, config.WaveformSamples)];
    [decodedFrame, valid, reason] = robot_waveform_decode(received);

    verifyTrue(testCase, valid, reason);
    verifyEqual(testCase, decodedFrame, frame);
end
end

function testHfmFskFramingMatchesConfiguredLengths(testCase)
config = robot_waveform_config();
frame = robot_packet_encode([0, 90, 120, 180], [0, 0, 0], 3, 4);
signal = robot_waveform_modulate(frame);

verifyEqual(testCase, config.HfmSamples, 1200);
verifyEqual(testCase, config.SamplesPerBit, 96);
verifyEqual(testCase, config.GuardSamples, 240);
verifyEqual(testCase, numel(signal), config.WaveformSamples);
verifyEqual(testCase, config.MinimumDecodeSamples, ...
    config.WaveformSamples - config.GuardSamples);
verifyEqual(testCase, config.MinimumDecodeSamples, ...
    config.HfmSamples + config.GuardSamples + config.PayloadSamples);
verifyEqual(testCase, config.BitRate, 500);
end

function testRingBufferWrapsAndReportsOverflow(testCase)
buffer = ByteRingBuffer(5);
buffer.push(uint8([1, 2, 3, 4]));
buffer.discard(3);
buffer.push(uint8([5, 6, 7, 8]));
verifyEqual(testCase, buffer.peek(5), uint8([4, 5, 6, 7, 8]));

buffer.push(uint8(9));
verifyEqual(testCase, buffer.DroppedBytes, 1);
verifyEqual(testCase, buffer.peek(5), uint8([5, 6, 7, 8, 9]));

buffer.push(uint8(1:8));
verifyEqual(testCase, buffer.DroppedBytes, 9);
verifyEqual(testCase, buffer.peek(5), uint8([4, 5, 6, 7, 8]));
end

function testSampleRingBufferPreservesFractionalSamples(testCase)
buffer = SampleRingBuffer(5);
buffer.push([0.125, 0.25, 0.375, 0.5]);
buffer.discard(3);
buffer.push([0.625, 0.75, 0.875, 1]);

verifyEqual(testCase, buffer.peek(5), ...
    single([0.5, 0.625, 0.75, 0.875, 1]));
verifyEqual(testCase, buffer.DroppedSamples, 0);
end
