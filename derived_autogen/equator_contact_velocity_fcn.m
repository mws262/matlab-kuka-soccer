function equator_contact_velocity = equator_contact_velocity_fcn(ax,ay,vx,vy)
%EQUATOR_CONTACT_VELOCITY_FCN
%    EQUATOR_CONTACT_VELOCITY = EQUATOR_CONTACT_VELOCITY_FCN(AX,AY,VX,VY)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    07-Aug-2018 13:41:58

t2 = abs(ax);
t3 = abs(ay);
t4 = t2.^2;
t5 = t3.^2;
t6 = t4+t5;
t7 = 1.0./sqrt(t6);
equator_contact_velocity = [vx,vy,ax.*t7.*vx+ay.*t7.*vy];
