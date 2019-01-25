function out1 = full_contact_velocity_fcn(R,ax,ay,jx,jy,thetadot,theta,vx,vy)
%FULL_CONTACT_VELOCITY_FCN
%    OUT1 = FULL_CONTACT_VELOCITY_FCN(R,AX,AY,JX,JY,THETADOT,THETA,VX,VY)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    25-Jan-2019 13:05:21

t2 = ax.^2;
t3 = ay.^2;
t4 = t2+t3;
t5 = sin(theta);
t6 = cos(theta);
t7 = 1.0./t4.^(3.0./2.0);
t8 = t4.^(3.0./2.0);
out1 = [t7.*(t8.*vx-R.*jx.*t3.*t6+R.*ax.*ay.*jy.*t6+R.*ax.*t2.*t5.*thetadot+R.*ax.*t3.*t5.*thetadot),t7.*(t8.*vy-R.*jy.*t2.*t6+R.*ax.*ay.*jx.*t6+R.*ay.*t2.*t5.*thetadot+R.*ay.*t3.*t5.*thetadot),R.*t6.*thetadot];