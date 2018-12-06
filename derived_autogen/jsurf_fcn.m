function out1 = jsurf_fcn(ax,ay,theta)
%JSURF_FCN
%    OUT1 = JSURF_FCN(AX,AY,THETA)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    06-Dec-2018 10:17:50

t2 = cos(theta);
t3 = abs(t2);
t4 = 1.0./t3;
t5 = theta.*2.0;
t6 = sin(t5);
t7 = ax.^2;
t8 = ay.^2;
t9 = t7+t8;
t10 = 1.0./sqrt(t9);
out1 = [(ax.*t4.*t6.*t10)./2.0,(ay.*t4.*t6.*t10)./2.0,t2.^2.*t4];
