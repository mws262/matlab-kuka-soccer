function exceptions = test_spline_concat_in_dimension()
%TEST_SPLINE_CONCAT_IN_DIMENSION
tolerance = 1e-15;

theader('Testing spline_concat_in_dimension.');
exceptions = {};
exceptions{end+1} = do_test(@two_d);
exceptions{end+1} = do_test(@three_d);
exceptions{end+1} = do_test(@bad_cases);


% Only two dimensions of splines are given. Other dimension always
% evaluates to 0.
    function two_d()
        tname('Two given dimensions.');

        breaks = linspace(-1, 5, 10);
        knots = [    0.5669    2.7555         0
            -3.4350    2.3427         0
            0.6206   -0.6972         0
            1.9480    1.9375         0
            -0.7354    4.4521         0
            3.3627    2.8423         0
            2.3139    2.0557         0
            -1.3997   -3.9067         0
            -0.4579   -1.1007         0
            -1.1361    0.9090         0];
        
        xspline = spline(breaks, knots(:,1));
        yspline = spline(breaks, knots(:,2));
        concat_spline = spline_concat_in_dimension(xspline, yspline);
        total_spline = spline(breaks, knots');
        
        assert(concat_spline.dim == total_spline.dim);
        assert(concat_spline.order == total_spline.order);
        assert_near(concat_spline.breaks, total_spline.breaks, tolerance, 'Normal spline and concatenated spline do not have matching breaks.');
        assert_near(concat_spline.coefs, total_spline.coefs, tolerance, 'Normal spline and concatenated spline do not have matching coefficients.');

        assert_near(ppval(concat_spline, 25), ppval(total_spline, 25), tolerance, 'Normal and concatenated splines do not evaluate the same.');
        assert_near(ppval(concat_spline, -25), ppval(total_spline, -25), tolerance, 'Normal and concatenated splines do not evaluate the same.');
        assert_near(ppval(concat_spline, 0), ppval(total_spline, 0), tolerance, 'Normal and concatenated splines do not evaluate the same.');
    end

% All three dimensions worth of splines are given.
    function three_d()
        tname('Three given dimensions.');
        
        breaks = linspace(-1, 5, 10);
        knots = [-4.1704    3.3970   -0.5615
            1.6160    0.3262   -1.9982
            0.1698    0.5389   -0.9861
            -3.2895    1.8007    3.3336
            4.3856   -1.3281   -0.9637
            0.9048   -2.6071   -1.0982
            -0.5937    0.7892   -1.3955
            4.4192    3.6689   -3.5974
            1.5591   -0.9322   -2.3987
            -0.4805   -3.8738   -4.1318];
        
        xspline = spline(breaks, knots(:,1));
        yspline = spline(breaks, knots(:,2));
        zspline = spline(breaks, knots(:,3));
        concat_spline = spline_concat_in_dimension(xspline, yspline, zspline);
        total_spline = spline(breaks, knots');
        
        assert(concat_spline.dim == total_spline.dim);
        assert(concat_spline.order == total_spline.order);
        assert_near(concat_spline.breaks, total_spline.breaks, tolerance, 'Normal spline and concatenated spline do not have matching breaks.');
        assert_near(concat_spline.coefs, total_spline.coefs, tolerance, 'Normal spline and concatenated spline do not have matching coefficients.');

        assert_near(ppval(concat_spline, 25), ppval(total_spline, 25), tolerance, 'Normal and concatenated splines do not evaluate the same.');
        assert_near(ppval(concat_spline, -25), ppval(total_spline, -25), tolerance, 'Normal and concatenated splines do not evaluate the same.');
        assert_near(ppval(concat_spline, 0), ppval(total_spline, 0), tolerance, 'Normal and concatenated splines do not evaluate the same.');
    end

    function bad_cases()
        tname('Error cases.');
        breaks = linspace(-1, 5, 10);
        knots = [-4.1704    3.3970   -0.5615
            1.6160    0.3262   -1.9982
            0.1698    0.5389   -0.9861
            -3.2895    1.8007    3.3336
            4.3856   -1.3281   -0.9637
            0.9048   -2.6071   -1.0982
            -0.5937    0.7892   -1.3955
            4.4192    3.6689   -3.5974
            1.5591   -0.9322   -2.3987
            -0.4805   -3.8738   -4.1318];
        
        % Breaks must align.
        xspline = spline(breaks + 1e-5, knots(:,1));
        yspline = spline(breaks, knots(:,2));
        assert_error(@()spline_concat_in_dimension(xspline, yspline));
        
        % Splines must be single-dimensional
        xspline = spline(breaks, knots');
        yspline = spline(breaks, knots(:,2));
        assert_error(@()spline_concat_in_dimension(xspline, yspline));
        
        % Splines must be the same order.
        xspline = spline(breaks, knots(:,1));
        yspline = spline(breaks, knots(:,2));
        xspline.coefs = [ones(size(xspline.coefs,1),1), xspline.coefs];
        assert_error(@()spline_concat_in_dimension(xspline, yspline));
    end
end

