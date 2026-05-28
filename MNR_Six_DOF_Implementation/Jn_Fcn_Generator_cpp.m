%% ------------------------------------------------------------------------
% Generate C++ Jacobian Source File Automatically
% -------------------------------------------------------------------------

fileName = 'MNR_J.cpp';

fid = fopen(fileName, 'w');

%% ------------------------------------------------------------------------
% C++ Headers
% -------------------------------------------------------------------------

fprintf(fid, '/* Auto Generated Jacobian File */\n\n');

fprintf(fid, '#include <iostream>\n');
fprintf(fid, '#include <math.h>\n');
fprintf(fid, '#include <Eigen/Dense>\n');
fprintf(fid, '#include "MNR_J.h"\n\n');

fprintf(fid, 'using namespace std;\n');
fprintf(fid, 'using namespace Eigen;\n\n');

%% ------------------------------------------------------------------------
% Class Constructor
% -------------------------------------------------------------------------

fprintf(fid, 'MNR_J::MNR_J() {}\n\n');

%% ------------------------------------------------------------------------
% Function Header
% -------------------------------------------------------------------------

fprintf(fid, '\t/* The Main MNR Jacobian Matrix Function */\n');
fprintf(fid, 'Eigen::MatrixXd MNR_J::MNR_Jacobian(VectorXd Theta) {\n\n');

%% ------------------------------------------------------------------------
% Joint Variables
% -------------------------------------------------------------------------

fprintf(fid, '\t/* Joint Angles */\n');

for k = 1:n
    fprintf(fid, '\tdouble t%d = Theta(%d);\n', k, k-1);
end

fprintf(fid, '\n');

%% ------------------------------------------------------------------------
% Link Lengths
% -------------------------------------------------------------------------

fprintf(fid, '\t/* Link Lengths */\n');

numLengths = length(Lengths);

for k = 1:numLengths
    fprintf(fid, '\tdouble L%d = %.15g;\n', k, Lengths(k));
end

fprintf(fid, '\n');

%% ------------------------------------------------------------------------
% Cosines
% -------------------------------------------------------------------------

fprintf(fid, '\t/* Cos and Sin */\n');

for k = 1:n
    fprintf(fid, '\tdouble c%d = cos(t%d);\n', k, k);
end

fprintf(fid, '\n');

%% ------------------------------------------------------------------------
% Sines
% -------------------------------------------------------------------------

for k = 1:n
    fprintf(fid, '\tdouble s%d = sin(t%d);\n', k, k);
end

fprintf(fid, '\n');

%% ------------------------------------------------------------------------
% Jacobian Matrix Declaration
% -------------------------------------------------------------------------

[numRows, numCols] = size(Jn_sym);

fprintf(fid, '\t/* Jacobian Matrix */\n');
fprintf(fid, '\tMatrixXd J(%d, %d);\n\n', numRows, numCols);

%% ------------------------------------------------------------------------
% Write Jacobian Elements
% -------------------------------------------------------------------------

for i = 1:numRows

    for j = 1:numCols

        exprSym = Jn_sym(i,j);

        % Convert symbolic expression to string
        exprStr = char(exprSym);

        % Remove extra spaces/newlines
        exprStr = regexprep(exprStr, '\s+', ' ');

        % MATLAB symbolic occasionally writes .* ./ .^
        exprStr = strrep(exprStr, '.*', '*');
        exprStr = strrep(exprStr, './', '/');
        exprStr = strrep(exprStr, '.^', '^');

        % Write Eigen matrix assignment
        fprintf(fid, '\tJ(%d, %d) = %s;\n', ...
            i-1, j-1, exprStr);

    end

    fprintf(fid, '\n');

end

%% ------------------------------------------------------------------------
% Return Statement
% -------------------------------------------------------------------------

fprintf(fid, '\treturn J;\n\n');

fprintf(fid, '}\n');

fclose(fid);

disp(['C++ Jacobian source generated: ', fileName]);