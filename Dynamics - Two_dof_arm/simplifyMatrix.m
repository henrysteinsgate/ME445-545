function MatrixOut = simplifyMatrix(MatrixIn)

s = vpa(MatrixIn,6); % Convert to decimal to n places
k = char(s); % Convert to a string
sc = strfind(k,'e-'); % Find all the e- terms
if length(sc) >= 1 % If there’s an e- term
 k = strrep(k,k(sc(1):sc(1)+1),'*0*'); % Multiply that term by 0
end
warning off
MatrixOut = vpa(simplify(sym(k)),6); % Convert to symbolic expression
warning on % 2 decimal places