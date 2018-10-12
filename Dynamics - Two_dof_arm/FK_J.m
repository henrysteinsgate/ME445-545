function Tnew = fwdkin(theta,num)

a = [0 0.475 0.325];
alpha = [pi/2 0 0];
d = [0.175 0 0];

n = length(a);

T0n = 1;

for i = 1:num
    T0n = T0n * [cos(theta(i)), -sin(theta(i))*cos(alpha(i)), sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i));
    sin(theta(i)), cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i));
    0, sin(alpha(i)), cos(alpha(i)), d(i);
    0, 0, 0, 1];
end

s = vpa(T0n,6); % Convert to decimal to n places
k = char(s); % Convert to a string
sc = strfind(k,'e-'); % Find all the e- terms
if length(sc) >= 1 % If there’s an e- term
 k = strrep(k,k(sc(1):sc(1)+1),'*0*'); % Multiply that term by 0
end
warning off
Tnew = vpa(simplify(sym(k)),2); % Convert to symbolic expression
warning on % 2 decimal places

%% For quaterion 

% T = T0n;
% Ex=0.5*sign(T(3,2)-T(2,3))*sqrt(T(1,1)-T(2,2)-T(3,3)+1);
% Ey=0.5*sign(T(1,3)-T(3,1))*sqrt(T(2,2)-T(3,3)-T(1,1)+1);
% Ez=0.5*sign(T(2,1)-T(1,2))*sqrt(T(3,3)-T(1,1)-T(2,2)+1);
% niu=0.5*sqrt(T(1,1)+T(2,2)+T(3,3)+1);
% 
% fprintf(' position: Point(x = %f , y = %f, z = %f),\n orientation: Quaternion(x = %f, y = %f, z = %f, w = %f)\n',...
%     T(1,4)/1000,T(2,4)/1000,T(3,4)/1000,Ex,Ey,Ez,niu);