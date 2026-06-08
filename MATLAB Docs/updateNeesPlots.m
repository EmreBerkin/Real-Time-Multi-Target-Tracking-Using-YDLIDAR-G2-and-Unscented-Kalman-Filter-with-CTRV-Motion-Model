function updateNeesPlots(figNEES, tracks, winLen, chiLo, chiHi)
    % NEES panelini güncelleyerek durum tahmin hatasını grafikleştirir.
    if ~isvalid(figNEES), return; end
    neesAxes = findobj(figNEES, 'Type', 'axes');
    
    for ti = 1:numel(tracks)
        tr = tracks{ti}; 
        if ~tr.confirmed || isempty(tr.neesHistory), continue; end
        
        slot = tr.plotSlot; 
        neesHist = tr.neesHistory; 
        Nnees = numel(neesHist);
        idxWin = max(1, Nnees - winLen + 1):Nnees; 
        yData = neesHist(idxWin); 
        xData = 1:numel(yData);

        hAx = neesAxes(7 - slot);
        hLine = findobj(hAx, 'Type', 'line', '-and', 'Marker', 'o');
        hWarn = findobj(hAx, 'Type', 'line', '-and', 'Marker', 'x');

        if ~isempty(hLine), set(hLine, 'XData', xData, 'YData', yData, 'Color', tr.color); end
        
        outMask = ((yData < chiLo) | (yData > chiHi)) & ~isnan(yData) & ~isinf(yData);
        if any(outMask)
            if ~isempty(hWarn), set(hWarn, 'XData', xData(outMask), 'YData', yData(outMask), 'Color', [1 0.9 0.1]); end
        else
            if ~isempty(hWarn), set(hWarn, 'XData', nan, 'YData', nan); end
        end

        validY = yData(~isnan(yData) & ~isinf(yData));
        if isempty(validY), pctOK = 0; else, pctOK = 100 * mean((validY >= chiLo) & (validY <= chiHi)); end
        
        if tr.missed > 0
            statusStr = sprintf('T%d NEES | miss=%d | OK %.0f%%', tr.id, tr.missed, pctOK); 
        else
            statusStr = sprintf('T%d NEES | ACTIVE | OK %.0f%%', tr.id, pctOK); 
        end
        title(hAx, statusStr, 'Color', tr.color); 
        xlim(hAx, [0 winLen]);
        if isempty(validY)
            ylim(hAx, [0 max(chiHi * 2.5, 25)]); 
        else
            ylim(hAx, [0 max([chiHi * 2.5, 25, max(validY) * 1.15])]); 
        end
    end
end
