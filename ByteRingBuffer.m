classdef ByteRingBuffer < handle
    %BYTERINGBUFFER Fixed-capacity circular buffer for uint8 audio samples.
    properties (SetAccess = private)
        Capacity
        Count = 0
        DroppedBytes = 0
    end

    properties (Access = private)
        Data
        Head = 1
        Tail = 1
    end

    methods
        function obj = ByteRingBuffer(capacity)
            validateattributes(capacity, {'numeric'}, ...
                {'scalar', 'integer', 'positive'});
            obj.Capacity = double(capacity);
            obj.Data = zeros(1, obj.Capacity, 'uint8');
        end

        function push(obj, bytes)
            bytes = uint8(bytes(:).');
            incoming = numel(bytes);
            if incoming == 0
                return;
            end

            if incoming >= obj.Capacity
                obj.DroppedBytes = obj.DroppedBytes + obj.Count + ...
                    incoming - obj.Capacity;
                obj.Data = bytes(end - obj.Capacity + 1:end);
                obj.Head = 1;
                obj.Tail = 1;
                obj.Count = obj.Capacity;
                return;
            end

            overflow = max(0, obj.Count + incoming - obj.Capacity);
            obj.Head = mod(obj.Head - 1 + overflow, obj.Capacity) + 1;
            obj.Count = obj.Count - overflow;
            obj.DroppedBytes = obj.DroppedBytes + overflow;
            indices = mod(obj.Tail - 1 + (0:incoming - 1), obj.Capacity) + 1;
            obj.Data(indices) = bytes;
            obj.Tail = mod(obj.Tail - 1 + incoming, obj.Capacity) + 1;
            obj.Count = obj.Count + incoming;
        end

        function bytes = peek(obj, count)
            validateattributes(count, {'numeric'}, ...
                {'scalar', 'integer', 'nonnegative', '<=', obj.Count});
            indices = mod((obj.Head - 1) + (0:count - 1), obj.Capacity) + 1;
            bytes = obj.Data(indices);
        end

        function discard(obj, count)
            validateattributes(count, {'numeric'}, ...
                {'scalar', 'integer', 'nonnegative', '<=', obj.Count});
            obj.Head = mod((obj.Head - 1) + count, obj.Capacity) + 1;
            obj.Count = obj.Count - count;
        end
    end
end
