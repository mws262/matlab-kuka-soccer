function out1 = contact_arc_fcn(R,ax,ay,rx,ry,theta)
%CONTACT_ARC_FCN
%    OUT1 = CONTACT_ARC_FCN(R,AX,AY,RX,RY,THETA)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    18-Feb-2019 09:13:48

t2 = abs(ax);
t3 = abs(ay);
t4 = cos(theta);
t5 = t2.^2;
t6 = t3.^2;
t7 = t5+t6;
t8 = 1.0./sqrt(t7);
out1 = [rx-R.*ax.*t4.*t8,ry-R.*ay.*t4.*t8,R+R.*sin(theta)];
