function out1 = pusher_linear_velocity_fcn(R,ax,ay,jx,jy,pusherX,pusherY,pusherZ,rx,ry,thetadot,theta,vx,vy)
%PUSHER_LINEAR_VELOCITY_FCN
%    OUT1 = PUSHER_LINEAR_VELOCITY_FCN(R,AX,AY,JX,JY,PUSHERX,PUSHERY,PUSHERZ,RX,RY,THETADOT,THETA,VX,VY)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    18-Feb-2019 09:13:49

t2 = sin(theta);
t3 = ax.^2;
t4 = ay.^2;
t5 = theta.*2.0;
t6 = sin(t5);
t7 = cos(theta);
t8 = t3+t4;
t9 = 1.0./t8.^(3.0./2.0);
t10 = R.*t2;
t11 = R-pusherZ+t10;
t12 = t7.^2;
t13 = ax.*jy;
t14 = t13-ay.*jx;
t15 = 1.0./t8;
t16 = 1.0./sqrt(t8);
t17 = R.*ax.*t7.*t16;
t18 = pusherX-rx+t17;
t19 = ax.*t3.*thetadot;
t20 = ax.*t4.*thetadot;
t21 = (ax.*ay.*jy.*t6)./2.0;
t22 = t19+t20+t21-(jx.*t4.*t6)./2.0;
t23 = R.*ay.*t7.*t16;
t24 = pusherY-ry+t23;
t25 = ay.*t4.*thetadot.*2.0;
t26 = ay.*t3.*thetadot.*2.0;
t27 = ax.*ay.*jx.*t6;
t28 = t25+t26+t27-jy.*t3.*t6;
out1 = [vx+t2.*vx-t9.*t11.*t22-t12.*t14.*t15.*t24,vy+t2.*vy-(t9.*t11.*t28)./2.0+t12.*t14.*t15.*t18,-t9.*t18.*t22-(t9.*t24.*t28)./2.0+ax.*t7.*t16.*vx+ay.*t7.*t16.*vy];
