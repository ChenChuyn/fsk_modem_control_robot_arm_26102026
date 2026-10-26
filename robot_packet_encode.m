function frame = robot_packet_encode(angles, gyro, sequence, session)
%ROBOT_PACKET_ENCODE Build a fixed-size SECDED- and CRC-protected robot frame.
% angles is four servo angles in integer degrees, 0..180.
% gyro is [x y z] in degrees/second, quantized to 0.01 degree/second.
% sequence and session are integers in the range 0..65535.

validateattributes(angles, {'numeric'}, ...
    {'vector', 'numel', 4, 'finite', 'integer', '>=', 0, '<=', 180});
if nargin == 2
    sequence = gyro;
    gyro = [0, 0, 0];
end
if nargin < 4
    session = 0;
end
validateattributes(gyro, {'numeric'}, ...
    {'vector', 'numel', 3, 'finite', '>=', -327.68, '<=', 327.67});
validateattributes(sequence, {'numeric'}, ...
    {'scalar', 'finite', 'integer', '>=', 0, '<=', 65535});
validateattributes(session, {'numeric'}, ...
    {'scalar', 'finite', 'integer', '>=', 0, '<=', 65535});

sync = uint8([hex2dec('D3'), hex2dec('91'), hex2dec('C5'), hex2dec('7A')]);
body = zeros(1, 15, 'uint8');
body(1) = uint8(1);
body(2) = uint8(bitshift(uint16(session), -8));
body(3) = uint8(bitand(uint16(session), uint16(255)));
body(4) = uint8(bitshift(uint16(sequence), -8));
body(5) = uint8(bitand(uint16(sequence), uint16(255)));

for k = 1:4
    body(k + 5) = uint8(angles(k));
end

for k = 1:3
    rate = int16(round(gyro(k)*100));
    encodedRate = typecast(rate, 'uint16');
    body(2*k + 8) = uint8(bitshift(encodedRate, -8));
    body(2*k + 9) = uint8(bitand(encodedRate, uint16(255)));
end

crc = crc16Ccitt(body);
protected = [body, uint8(bitshift(crc, -8)), uint8(bitand(crc, uint16(255)))];
encoded = zeros(1, 2*numel(protected), 'uint8');

for k = 1:numel(protected)
    code = encodeSecdedByte(protected(k));
    packed = uint16(sum(double(code) .* 2.^(12:-1:0)));
    packed = bitshift(packed, 3);
    encoded(2*k - 1) = uint8(bitshift(packed, -8));
    encoded(2*k) = uint8(bitand(packed, uint16(255)));
end

frame = [sync, encoded];
end

function code = encodeSecdedByte(value)
dataPositions = [3, 5, 6, 7, 9, 10, 11, 12];
code = zeros(1, 13);
code(dataPositions) = bitget(value, 8:-1:1);

for parityPosition = [1, 2, 4, 8]
    covered = bitand(1:12, parityPosition) ~= 0;
    code(parityPosition) = mod(sum(code(covered)), 2);
end

code(13) = mod(sum(code(1:12)), 2);
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
