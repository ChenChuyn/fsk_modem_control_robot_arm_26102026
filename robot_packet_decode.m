function [packet, valid, correctedBits, reason] = robot_packet_decode(frame)
%ROBOT_PACKET_DECODE Verify and decode one robot pose frame.
% SECDED corrects one flipped bit per encoded byte and detects two-bit errors.

sync = uint8([hex2dec('D3'), hex2dec('91'), hex2dec('C5'), hex2dec('7A')]);
bodyLength = 17;
frameLength = numel(sync) + 2*bodyLength;
packet = struct('sequence', [], 'angles', []);
valid = false;
correctedBits = 0;
reason = 'Invalid frame length';

if ~isa(frame, 'uint8') || numel(frame) ~= frameLength
    return;
end

frame = reshape(frame, 1, []);
reason = 'Synchronization word mismatch';
if ~isequal(frame(1:numel(sync)), sync)
    return;
end

decoded = zeros(1, bodyLength, 'uint8');
for k = 1:bodyLength
    packed = bitor(bitshift(uint16(frame(2*k + 3)), 8), ...
        uint16(frame(2*k + 4)));
    code = bitget(packed, 16:-1:4);
    [value, ok, corrected] = decodeSecdedByte(code);
    if ~ok
        reason = 'Uncorrectable SECDED error';
        return;
    end
    decoded(k) = value;
    correctedBits = correctedBits + double(corrected);
end

reason = 'Unsupported packet version';
if decoded(1) ~= 1
    return;
end

expectedCrc = bitor(bitshift(uint16(decoded(16)), 8), uint16(decoded(17)));
actualCrc = crc16Ccitt(decoded(1:15));
reason = 'CRC-16 mismatch';
if expectedCrc ~= actualCrc
    return;
end

angles = zeros(1, 4);
for k = 1:4
    value = double(decoded(k + 5));
    if value > 180
        reason = 'Joint angle outside 0..180 degrees';
        return;
    end
    angles(k) = value;
end

gyro = zeros(1, 3);
for k = 1:3
    value = bitor(bitshift(uint16(decoded(2*k + 8)), 8), ...
        uint16(decoded(2*k + 9)));
    gyro(k) = double(typecast(value, 'int16')) / 100;
end

packet.session = double(bitor(bitshift(uint16(decoded(2)), 8), uint16(decoded(3))));
packet.sequence = double(bitor(bitshift(uint16(decoded(4)), 8), uint16(decoded(5))));
packet.angles = angles;
packet.gyro = gyro;
valid = true;
reason = '';
end

function [value, ok, corrected] = decodeSecdedByte(code)
dataPositions = [3, 5, 6, 7, 9, 10, 11, 12];
syndrome = 0;
for parityPosition = [1, 2, 4, 8]
    covered = bitand(1:12, parityPosition) ~= 0;
    if mod(sum(code(covered)), 2) ~= 0
        syndrome = syndrome + parityPosition;
    end
end

overallParity = mod(sum(code), 2);
ok = true;
corrected = false;

if syndrome ~= 0 && overallParity == 1
    code(syndrome) = 1 - code(syndrome);
    corrected = true;
elseif syndrome == 0 && overallParity == 1
    code(13) = 1 - code(13);
    corrected = true;
elseif syndrome ~= 0
    ok = false;
    value = uint8(0);
    return;
end

value = uint8(sum(double(code(dataPositions)) .* 2.^(7:-1:0)));
end

function crc = crc16Ccitt(bytes)
crc = uint16(hex2dec('FFFF'));
polynomial = uint16(hex2dec('1021'));

for k = 1:numel(bytes)
    for bit = 7:-1:0
        inputBit = uint16(bitget(bytes(k), bit + 1));
        topBit = bitget(crc, 16);
        crc = bitshift(crc, 1);
        if xor(logical(topBit), logical(inputBit))
            crc = bitxor(crc, polynomial);
        end
    end
end
end
