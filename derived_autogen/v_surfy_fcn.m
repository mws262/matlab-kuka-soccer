function v_surfy = v_surfy_fcn(ax,ay,theta,vx,vy)
%V_SURFY_FCN
%    V_SURFY = V_SURFY_FCN(AX,AY,THETA,VX,VY)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    21-Aug-2018 10:15:52

t2 = cos(theta);
t3 = abs(ax);
t4 = abs(ay);
t5 = t3.^2;
t6 = t4.^2;
t7 = t5+t6;
t8 = 1.0./sqrt(t7);
t9 = abs(t2);
t10 = 1.0./t9;
t11 = theta.*2.0;
t12 = sin(t11);
t13 = sin(theta);
t14 = ax.^2;
t15 = ay.^2;
t16 = t14+t15;
t17 = 1.0./sqrt(t16);
v_surfy = t2.^2.*t10.*(ax.*t2.*t8.*vx+ay.*t2.*t8.*vy)+ax.*t10.*t12.*t17.*(vx+t13.*vx).*(1.0./2.0)+ay.*t10.*t12.*t17.*(vy+t13.*vy).*(1.0./2.0);
