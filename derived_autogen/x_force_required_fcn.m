function fax_solve = x_force_required_fcn(I,R,ax,m)
%X_FORCE_REQUIRED_FCN
%    FAX_SOLVE = X_FORCE_REQUIRED_FCN(I,R,AX,M)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    06-Dec-2018 10:17:47

fax_solve = 1.0./R.^2.*ax.*(I+R.^2.*m);
