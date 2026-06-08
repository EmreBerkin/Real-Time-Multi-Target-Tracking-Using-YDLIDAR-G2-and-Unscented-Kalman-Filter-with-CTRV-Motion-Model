function [x_frame, y_frame] = processLidarFrame(frameRangesMM, frameAnglesDeg, minRangeMM, maxRangeMM)
    % Milimetrik ham Lidar verilerini metreye çevirir ve kartezyen koordinatları üretir.
    r_frame = frameRangesMM / 1000; 
    th_frame = deg2rad(frameAnglesDeg);
    
    x_frame = r_frame .* cos(th_frame); 
    y_frame = r_frame .* sin(th_frame);
end
