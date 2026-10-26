classdef SampleRingBuffer < handle
    %SAMPLERINGBUFFER Fixed-capacity circular buffer for audio samples.
    properties (SetAccess = private)
        Capacity
        Count = 0
        DroppedSamples = 0
    end

    properties (Access = private)
        Data
        Head = 1
        Tail = 1
    end

    methods
        function obj = SampleRingBuffer(capacity)
            validateattributes(capacity, {'numeric'}, ...
                {'scalar', 'integer', 'positive'});
            obj.Capacity = double(capacity);
            obj.Data = zeros(1, obj.Capacity, 'single');
        end

        function push(obj, samples)
            samples = single(samples(:).');
            incoming = numel(samples);
            if incoming == 0
                return;
            end

            if incoming >= obj.Capacity
                obj.DroppedSamples = obj.DroppedSamples + obj.Count + ...
                    incoming - obj.Capacity;
                obj.Data = samples(end - obj.Capacity + 1:end);
                obj.Head = 1;
                obj.Tail = 1;
                obj.Count = obj.Capacity;
                return;
            end

            overflow = max(0, obj.Count + incoming - obj.Capacity);
            obj.Head = mod(obj.Head - 1 + overflow, obj.Capacity) + 1;
            obj.Count = obj.Count - overflow;
            obj.DroppedSamples = obj.DroppedSamples + overflow;
            indices = mod(obj.Tail - 1 + (0:incoming - 1), obj.Capacity) + 1;
            obj.Data(indices) = samples;
            obj.Tail = mod(obj.Tail - 1 + incoming, obj.Capacity) + 1;
            obj.Count = obj.Count + incoming;
        end

        function samples = peek(obj, count)
            validateattributes(count, {'numeric'}, ...
                {'scalar', 'integer', 'nonnegative', '<=', obj.Count});
            indices = mod((obj.Head - 1) + (0:count - 1), obj.Capacity) + 1;
            samples = obj.Data(indices);
        end

        function discard(obj, count)
            validateattributes(count, {'numeric'}, ...
                {'scalar', 'integer', 'nonnegative', '<=', obj.Count});
            obj.Head = mod((obj.Head - 1) + count, obj.Capacity) + 1;
            obj.Count = obj.Count - count;
        end
    end
end
