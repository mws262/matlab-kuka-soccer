function fay_solve = y_force_required_fcn(I,R,ay,m)
%Y_FORCE_REQUIRED_FCN
%    FAY_SOLVE = Y_FORCE_REQUIRED_FCN(I,R,AY,M)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    07-Aug-2018 13:41:58

fay_solve = 1.0./R.^2.*ay.*(I+R.^2.*m);
