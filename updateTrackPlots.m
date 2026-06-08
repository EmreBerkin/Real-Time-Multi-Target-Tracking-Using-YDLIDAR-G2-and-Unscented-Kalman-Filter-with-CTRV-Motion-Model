function updateTrackPlots(tracks, omegaThresh, showUncertainty, ellipseScale)
    % Aktif ve onaylanmış tüm izleri (tracks) ana lidar figüründe günceller.
    for t = 1:numel(tracks)
        tr = tracks{t};
        if ~tr.confirmed
            hideTrackHandles(tr); 
            continue; 
        end

        px = tr.x(1); py = tr.x(2); v = tr.x(3); psi = tr.x(4); omega = tr.x(5);
        vx = v * cos(psi); vy = v * sin(psi); speed = abs(v);

        if speed > 0.08, motionStr = sprintf('MOVING %.2fm/s', speed); else, motionStr = 'STATIONARY'; end
        if abs(omega) > omegaThresh, motionStr = [motionStr sprintf(' w=%.2f', omega)]; end
        if tr.missed > 0, motionStr = [motionStr sprintf(' | miss=%d', tr.missed)]; end

        if isfield(tr, 'lastBBox') && ~isempty(tr.lastBBox)
            bbox = tr.lastBBox; 
            if ishandle(tr.hBBox), set(tr.hBBox, 'Position', bbox, 'Visible', 'on'); end
            labelPos = [bbox(1), bbox(2) + bbox(4) + 0.08];
        else
            labelPos = [px, py + 0.15];
        end

        if ishandle(tr.hCtr), set(tr.hCtr, 'XData', px, 'YData', py, 'Visible', 'on'); end
        if ishandle(tr.hTxt), set(tr.hTxt, 'Position', [labelPos 0], 'String', sprintf('T%d | %s', tr.id, motionStr), 'Visible', 'on'); end
        if ishandle(tr.hVel), set(tr.hVel, 'XData', [px, px + vx * 0.4], 'YData', [py, py + vy * 0.4], 'Visible', 'on'); end
        if ishandle(tr.hTrail) && size(tr.history, 1) > 1
            set(tr.hTrail, 'XData', tr.history(:, 1), 'YData', tr.history(:, 2), 'Visible', 'on'); 
        end

        if ishandle(tr.hEllipse)
            if showUncertainty
                ellXY = covEllipse(tr.P(1:2, 1:2), [px; py], ellipseScale, 32);
                set(tr.hEllipse, 'XData', ellXY(1, :), 'YData', ellXY(2, :), 'Visible', 'on');
            else
                set(tr.hEllipse, 'XData', nan, 'YData', nan, 'Visible', 'off');
            end
        end
    end
end
