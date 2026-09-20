function numMatrix = extractNumericMatrix(rawCell)
    % Scans each row of rawCell and extracts rows that consist purely of numbers
    numRows = size(rawCell, 1);
    numCols = size(rawCell, 2);
    
    validRows = false(numRows, 1);
    
    for r = 1:numRows
        % Check if all non-missing items in row r are numeric
        rowVals = rawCell(r, :);
        isNum = cellfun(@(x) isnumeric(x) && ~isnan(x), rowVals);
        
        % Keep rows that have numeric elements and match the predominant column layout
        if any(isNum) && ~any(cellfun(@ischar, rowVals) | cellfun(@isstring, rowVals))
            validRows(r) = true;
        end
    end
    
    % Extract valid cell rows
    cleanCell = rawCell(validRows, :);
    
    % Handle missing elements or uneven rows by converting to double
    if isempty(cleanCell)
        numMatrix = [];
        return;
    end
    
    % Convert to matrix safely
    numMatrix = double(string(cleanCell));
    
    % Strip columns that are completely NaN (e.g. from trailing delimiters)
    numMatrix = numMatrix(:, ~all(isnan(numMatrix), 1));
end