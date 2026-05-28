%% ------------------------------------------------------------------------
% Generate MATLAB Jacobian function automatically
% -------------------------------------------------------------------------

fileName = 'MNR_Jn_.m';

fid = fopen(fileName, 'w');

%% Function header
fprintf(fid, 'function J = MNR_Jn_(Theta)\n\n');

%% ------------------------------------------------------------------------
% Link lengths
% -------------------------------------------------------------------------

numLengths = length(Lengths);

for k = 1:numLengths
    fprintf(fid, 'L%d = %.15g;\n', k, Lengths(k));
end

fprintf(fid, '\n');

%% ------------------------------------------------------------------------
% Joint variables
% -------------------------------------------------------------------------

for k = 1:n
    fprintf(fid, 't%d = Theta(%d);\n', k, k);
end

fprintf(fid, '\n');

%% ------------------------------------------------------------------------
% Cosines
% -------------------------------------------------------------------------

for k = 1:n
    fprintf(fid, 'c%d = cos(t%d);\n', k, k);
end

fprintf(fid, '\n');

%% ------------------------------------------------------------------------
% Sines
% -------------------------------------------------------------------------

for k = 1:n
    fprintf(fid, 's%d = sin(t%d);\n', k, k);
end

fprintf(fid, '\n');

fprintf(fid, '%% Jacobian\n\n');

%% ------------------------------------------------------------------------
% Write Jacobian elements
% -------------------------------------------------------------------------

[numRows, numCols] = size(Jn_sym);

for i = 1:numRows

    for j = 1:numCols

        exprSym = Jn_sym(i,j);

        % Variable name
        varName = sprintf('J%02d%02d', i, j);

        % Convert symbolic expression to string
        exprStr = char(exprSym);

        % Remove line breaks / extra spaces
        exprStr = regexprep(exprStr, '\s+', ' ');

        % Write expression
        fprintf(fid, '%s = %s;\n', varName, exprStr);

    end
end

% for i = 1:numRows
% 
%     for j = 1:numCols
% 
%         exprSym = Jn_sym(i,j);
% 
%         % Convert symbolic expression to string
%         exprStr = char(exprSym);
% 
%         % Remove line breaks / extra spaces
%         exprStr = regexprep(exprStr, '\s+', ' ');
% 
%         % Write directly into matrix form
%         fprintf(fid, 'J(%d, %d) = %s;\n', i-1, j-1, exprStr);
% 
%     end
% end

fprintf(fid, '\n');

%% ------------------------------------------------------------------------
% Construct Jacobian matrix
% -------------------------------------------------------------------------

fprintf(fid, 'J = [ ...\n');

for i = 1:numRows

    for j = 1:numCols

        varName = sprintf('J%02d%02d', i, j);

        if j < numCols
            fprintf(fid, '%s, ', varName);
        else
            fprintf(fid, '%s', varName);
        end

    end

    if i < numRows
        fprintf(fid, '; ...\n');
    else
        fprintf(fid, '];\n');
    end

end

fprintf(fid, '\nend\n');

fclose(fid);

disp(['MATLAB function generated: ', fileName]);