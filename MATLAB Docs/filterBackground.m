function [measurements, measBBoxes] = filterBackground(clusterList, backgroundCenters, bgMatchM)
    % Statik haritaya (arka plana) uyan kümeleri eler, dinamik hedefleri döndürür.
    measurements = []; 
    measBBoxes = {};
    
    for k = 1:numel(clusterList)
        ctr = clusterList{k}.center; 
        isBg = false;
        
        if ~isempty(backgroundCenters)
            if any(vecnorm(backgroundCenters - ctr, 2, 2) < bgMatchM)
                isBg = true; 
            end
        end
        
        if ~isBg
            measurements(end+1, :) = ctr; %#ok<AGROW>
            measBBoxes{end+1} = clusterList{k}.bbox; %#ok<AGROW>
        end
    end
end
