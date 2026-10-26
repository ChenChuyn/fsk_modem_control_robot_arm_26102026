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
