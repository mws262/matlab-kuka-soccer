function fay_solve = y_force_required_fcn(I,R,ay,m)
%Y_FORCE_REQUIRED_FCN
%    FAY_SOLVE = Y_FORCE_REQUIRED_FCN(I,R,AY,M)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    09-Jan-2019 09:46:53

fay_solve = 1.0./R.^2.*ay.*(I+R.^2.*m);
