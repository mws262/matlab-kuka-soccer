function v_surfx = v_surfx_fcn(ax,ay,theta,vx,vy)
%V_SURFX_FCN
%    V_SURFX = V_SURFX_FCN(AX,AY,THETA,VX,VY)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    21-Aug-2018 10:15:52

t2 = cos(theta);
t3 = abs(t2);
t4 = 1.0./t3;
t5 = sin(theta);
t6 = ax.^2;
t7 = ay.^2;
t8 = t6+t7;
t9 = 1.0./sqrt(t8);
v_surfx = -ay.*t2.*t4.*t9.*(vx+t5.*vx)+ax.*t2.*t4.*t9.*(vy+t5.*vy);
