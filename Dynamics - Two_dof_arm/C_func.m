function C = C_func(o1,o2)
%C_FUNC
%    C = C_FUNC(O1,O2)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    13-Aug-2018 12:49:34

t2 = o2.*2.0;
C = reshape([0.0,sin(t2).*4.93606e-2,sin(o1.*4.0).*(cos(t2)-3.0).*6.25e-9,0.0],[2,2]);
