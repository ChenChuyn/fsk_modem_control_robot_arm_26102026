function robot_arm_control(role)
%ROBOT_ARM_CONTROL Open a 3-D master or slave GUI for the simulated robot arm.
%   robot_arm_control("master") opens the pose and gyro controls.
%   robot_arm_control("slave") receives the modem through the default audio input.
%   Run a second MATLAB process for the other endpoint.

if nargin < 1 || isempty(role)
    role = "master";
end
role = lower(string(role));
if ~any(role == ["master", "slave"])
    error('robot_arm_control:InvalidRole', 'Role must be "master" or "slave".');
end

angles = [90, 90, 90, 45];
gyro = [0, 0, 0];
sequence = 0;
lastSequence = [];
lastSession = [];
lastReportedDrops = 0;
config = robot_waveform_config();
player = [];
pendingWaveform = [];
closing = false;
fig = uifigure('Name', sprintf('Robot arm - %s', role), ...
    'Position', [100, 100, 1000, 650]);
fig.CloseRequestFcn = @closeApp;

layout = uigridlayout(fig, [1, 2]);
layout.ColumnWidth = {310, '1x'};
layout.RowHeight = {'1x'};
layout.Padding = [12, 12, 12, 12];
layout.ColumnSpacing = 16;

controls = uigridlayout(layout, [9, 2]);
controls.Layout.Row = 1;
controls.Layout.Column = 1;
controls.RowHeight = {28, 28, 28, 28, 28, 28, 28, 28, '1x'};
controls.ColumnWidth = {120, '1x'};

if role == "master"
    uilabel(controls, 'Text', 'Audio output', 'HorizontalAlignment', 'right');
    outputLabel = uilabel(controls, 'Text', 'Default device');
    outputLabel.Layout.Column = 2;

    labels = {'Base (deg)', 'Shoulder (deg)', 'Elbow (deg)', 'Gripper (deg)'};
    sliders = gobjects(1, 4);
    for k = 1:4
        uilabel(controls, 'Text', labels{k}, 'HorizontalAlignment', 'right');
        sliders(k) = uislider(controls, 'Limits', [0, 180], ...
            'Value', angles(k), 'MajorTicks', [0, 90, 180]);
        sliders(k).ValueChangingFcn = ...
            @(source, event) updateMasterPose(k, event);
        sliders(k).ValueChangedFcn = ...
            @(source, event) updateMasterPose(k, event);
    end

    gyroLabels = {'Gyro X (deg/s)', 'Gyro Y (deg/s)', 'Gyro Z (deg/s)'};
    gyroFields = gobjects(1, 3);
    for k = 1:3
        uilabel(controls, 'Text', gyroLabels{k}, 'HorizontalAlignment', 'right');
        gyroFields(k) = uieditfield(controls, 'numeric', 'Value', 0, ...
            'Limits', [-327.68, 327.67]);
        gyroFields(k).ValueChangedFcn = @updateMasterGyro;
    end

    status = uilabel(controls, 'Text', 'Idle - no movement; no transmission.', ...
        'WordWrap', 'on');
    status.Layout.Row = 9;
    status.Layout.Column = [1, 2];
    lastSentState = [angles, gyro];
    sessionId = randi([0, 65535]);
    transmitTimer = timer('ExecutionMode', 'fixedSpacing', ...
        'Period', config.WaveformSamples/config.SampleRate, ...
        'BusyMode', 'drop', 'TimerFcn', @sendPoseIfChanged);
else
    uilabel(controls, 'Text', 'Audio input', 'HorizontalAlignment', 'right');
    inputLabel = uilabel(controls, 'Text', 'Default device');
    inputLabel.Layout.Column = 2;
    labels = {'Base (deg)', 'Shoulder (deg)', 'Elbow (deg)', 'Gripper (deg)'};
    angleLabels = gobjects(1, 4);
    for k = 1:4
        row = k + 1;
        uilabel(controls, 'Text', labels{k}, 'HorizontalAlignment', 'right');
        angleLabels(k) = uilabel(controls, 'Text', sprintf('%d', angles(k)));
        angleLabels(k).Layout.Row = row;
        angleLabels(k).Layout.Column = 2;
    end
    gyroLabels = {'Gyro X (deg/s)', 'Gyro Y (deg/s)', 'Gyro Z (deg/s)'};
    gyroReadouts = gobjects(1, 3);
    for k = 1:3
        row = k + 5;
        uilabel(controls, 'Text', gyroLabels{k}, 'HorizontalAlignment', 'right');
        gyroReadouts(k) = uilabel(controls, 'Text', '0.00');
        gyroReadouts(k).Layout.Row = row;
        gyroReadouts(k).Layout.Column = 2;
    end
    status = uilabel(controls, 'Text', 'Waiting for HFM-synchronized frames...', ...
        'WordWrap', 'on');
    status.Layout.Row = 9;
    status.Layout.Column = [1, 2];
    ring = ByteRingBuffer(131072);
    recorder = audiorecorder(config.SampleRate, 16, 1);
    lastAudioIndex = 0;
    record(recorder);
    audioTimer = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.05, ...
        'BusyMode', 'drop', 'TimerFcn', @receiveAudio);
end

axesHandle = uiaxes(layout);
axesHandle.Layout.Row = 1;
axesHandle.Layout.Column = 2;
axesHandle.XLim = [-1.8, 1.8];
axesHandle.YLim = [-1.8, 1.8];
axesHandle.ZLim = [0, 2.2];
axesHandle.DataAspectRatio = [1, 1, 1];
axesHandle.XGrid = 'on';
axesHandle.YGrid = 'on';
axesHandle.ZGrid = 'on';
axesHandle.View = [35, 24];
axesHandle.Title.String = '3-D robot arm pose';
axesHandle.XLabel.String = 'X';
axesHandle.YLabel.String = 'Y';
axesHandle.ZLabel.String = 'Z';
armLine = plot3(axesHandle, nan, nan, nan, '-o', 'LineWidth', 8, ...
    'MarkerSize', 10, 'MarkerFaceColor', [0.2, 0.55, 0.85]);
hold(axesHandle, 'on');
gripperLine = plot3(axesHandle, nan, nan, nan, '-', 'LineWidth', 4, ...
    'Color', [0.85, 0.35, 0.2]);
baseHeight = 0.12;
baseAngle = linspace(0, 2*pi, 36);
fill3(axesHandle, 0.22*cos(baseAngle), 0.22*sin(baseAngle), ...
    baseHeight*ones(size(baseAngle)), [0.35, 0.35, 0.4], ...
    'EdgeColor', 'none');
hold(axesHandle, 'off');
drawArm();
if role == "slave"
    start(audioTimer);
else
    start(transmitTimer);
end

    function updateMasterPose(joint, event)
        angles(joint) = round(event.Value);
        drawArm();
    end

    function updateMasterGyro(~, ~)
        gyro = arrayfun(@(field) field.Value, gyroFields);
    end

    function sendPoseIfChanged(~, ~)
        if ~isvalid(fig)
            return;
        end
        state = [angles, round(gyro*100)/100];
        if isequal(state, lastSentState)
            return;
        end

        packet = robot_packet_encode(angles, gyro, sequence, sessionId);
        waveform = robot_waveform_modulate(packet);
        pendingWaveform = waveform;
        status.Text = sprintf('Queued acoustic frame %u | servo [%s] | gyro [%s] deg/s', ...
            sequence, num2str(angles), num2str(gyro, '%.2f '));
        lastSentState = state;
        sequence = mod(sequence + 1, 65536);
        startNextTransmission();
    end

    function startNextTransmission()
        if closing || isempty(pendingWaveform) || ~isempty(player) || ~isvalid(fig)
            return;
        end
        waveform = pendingWaveform;
        pendingWaveform = [];
        player = audioplayer(waveform, config.SampleRate, 16);
        player.StopFcn = @finishTransmission;
        status.Text = 'Transmitting acoustic waveform...';
        play(player);
    end

    function finishTransmission(~, ~)
        player = [];
        if isempty(pendingWaveform)
            status.Text = 'Idle - no movement; no transmission.';
        else
            startNextTransmission();
        end
    end

    function appendAudioSamples(audio)
        if numel(audio) <= lastAudioIndex
            return;
        end
        newSamples = audio(lastAudioIndex + 1:end);
        pcm = uint8(round((min(1, max(-1, newSamples)) + 1)*127.5));
        ring.push(pcm);
        lastAudioIndex = numel(audio);
    end

    function receiveAudio(~, ~)
        audio = getaudiodata(recorder);
        appendAudioSamples(audio);

        if lastAudioIndex >= 2*config.SampleRate
            stop(recorder);
            appendAudioSamples(getaudiodata(recorder));
            recorder = audiorecorder(config.SampleRate, 16, 1);
            lastAudioIndex = 0;
            record(recorder);
        end

        while ring.Count >= config.WaveformSamples
            waveform = double(ring.peek(ring.Count))/127.5 - 1;
            [frame, valid, reason, confidence, syncStart] = ...
                robot_waveform_decode(waveform);
            if ~valid
                if strcmp(reason, 'HFM synchronization not found')
                    ring.discard(max(1, ring.Count - config.WaveformSamples + 1));
                else
                    ring.discard(config.WaveformSamples);
                end
                status.Text = ['Rejected waveform: ', reason];
                continue;
            end

            [decoded, packetValid, corrected, reason] = robot_packet_decode(frame);
            ring.discard(syncStart - 1 + config.WaveformSamples);
            if ~packetValid
                status.Text = ['Rejected packet: ', reason];
                continue;
            end

            if ~isempty(lastSession) && decoded.session == lastSession
                sequenceDelta = mod(decoded.sequence - lastSequence, 65536);
                if sequenceDelta == 0 || sequenceDelta >= 32768
                    status.Text = sprintf('Ignored stale sequence %u.', ...
                        decoded.sequence);
                    continue;
                end
            end
            lastSession = decoded.session;
            lastSequence = decoded.sequence;
            angles = decoded.angles;
            gyro = decoded.gyro;
            for joint = 1:4
                angleLabels(joint).Text = sprintf('%d', angles(joint));
            end
            for axis = 1:3
                gyroReadouts(axis).Text = sprintf('%.2f', gyro(axis));
            end
            drawArm();
            status.Text = sprintf('RX seq %u | HFM %.2f | corrected %d bit(s)', ...
                decoded.sequence, confidence, corrected);
        end

        if ring.DroppedBytes > lastReportedDrops
            status.Text = sprintf('Sample ring overflow: dropped %d sample(s).', ...
                ring.DroppedBytes);
            lastReportedDrops = ring.DroppedBytes;
        end
    end

    function drawArm()
        yaw = deg2rad(angles(1) - 90);
        shoulder = deg2rad(angles(2) - 90);
        elbow = shoulder + deg2rad(angles(3) - 90);
        lengths = [0.65, 0.55, 0.4];
        radial = [0, lengths(1)*cos(shoulder), ...
            lengths(1)*cos(shoulder) + lengths(2)*cos(elbow), ...
            lengths(1)*cos(shoulder) + lengths(2)*cos(elbow) + ...
            lengths(3)*cos(elbow)];
        z = [baseHeight, baseHeight + lengths(1)*sin(shoulder), ...
            baseHeight + lengths(1)*sin(shoulder) + lengths(2)*sin(elbow), ...
            baseHeight + lengths(1)*sin(shoulder) + lengths(2)*sin(elbow) + ...
            lengths(3)*sin(elbow)];
        x = radial*cos(yaw);
        y = radial*sin(yaw);
        armLine.XData = x;
        armLine.YData = y;
        armLine.ZData = z;

        opening = 0.04 + 0.14*angles(4)/180;
        forward = [cos(elbow)*cos(yaw), cos(elbow)*sin(yaw), sin(elbow)];
        lateral = [-sin(yaw), cos(yaw), 0];
        tip = [x(end), y(end), z(end)];
        jawA = tip + opening*lateral + 0.18*forward;
        jawB = tip - opening*lateral + 0.18*forward;
        gripperLine.XData = [tip(1), jawA(1), nan, tip(1), jawB(1)];
        gripperLine.YData = [tip(2), jawA(2), nan, tip(2), jawB(2)];
        gripperLine.ZData = [tip(3), jawA(3), nan, tip(3), jawB(3)];
        drawnow limitrate;
    end

    function closeApp(~, ~)
        closing = true;
        if exist('transmitTimer', 'var') && isvalid(transmitTimer)
            stop(transmitTimer);
            delete(transmitTimer);
        end
        if exist('audioTimer', 'var') && isvalid(audioTimer)
            stop(audioTimer);
            delete(audioTimer);
        end
        if exist('recorder', 'var') && isrecording(recorder)
            stop(recorder);
        end
        if ~isempty(player) && isplaying(player)
            stop(player);
        end
        delete(fig);
    end
end
