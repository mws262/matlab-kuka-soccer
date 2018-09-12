function exceptions = test_modified_gram_schmidt()
% TEST_MODIFIED_GRAM_SCHMIDT Unit tests for modified_gram_schmidt.
%
%   exceptions = TEST_MODIFIED_GRAM_SCHMIDT()
%
%   Inputs: <none>
%   Outputs:
%       `exceptions` -- Cell array of exceptions generated while running
%       sub-tests.
%
%   See also MODIFIED_GRAM_SCHMIDT, TEST_ALL.
%
tolerance = 1e-12;

theader('Testing modified_gram_schmidt.');

%% Set up an example
rot = axang2rotm([1, -4, 5, 1]);

bad_identity =    [1.0145    0.0086    0.0134
    0.0225    0.9969   -0.0166
    -0.0086    0.0167    1.0181];

bad_rot = bad_identity*rot;
fixed_rot = modified_gram_schmidt(bad_rot);

%% Run tests
exceptions = {};
exceptions{end+1} = do_test(@()test_unit_length(fixed_rot));
exceptions{end+1} = do_test(@()test_orthogonality(fixed_rot));
exceptions{end+1} = do_test(@()test_similar_to_original(rot, fixed_rot));
exceptions{end+1} = do_test(@()test_errors);

    function test_unit_length(corrected_rotation)
        tname('Unit length of columns');
        assert_near(sum(corrected_rotation.*corrected_rotation,1), ones(1,3), tolerance, 'Fixed rotation should have unit-length columns.');
    end
    function test_orthogonality(corrected_rotation)
        tname('Orthogonality of columns');
        
        assert_near(cross(corrected_rotation(:,1), corrected_rotation(:,2)), corrected_rotation(:,3), tolerance, 'Fixed rotation should have orthogonal columns.');
        assert_near(cross(corrected_rotation(:,2), corrected_rotation(:,3)), corrected_rotation(:,1), tolerance, 'Fixed rotation should have orthogonal columns.');
    end
    function test_similar_to_original(original_rotation, corrected_rotation)
        tname('Close to intended rotation');
        
        assert_near(original_rotation, corrected_rotation, 1e-1, 'Fixed rotation should be pretty close to the original un-messed up one.');
    end
    function test_errors()
        tname('Expected exceptions');
        assert_error(@()modified_gram_schmidt(ones(3,1)));
        assert_error(@()modified_gram_schmidt(ones(1,3)));
        assert_error(@()modified_gram_schmidt(ones(5,3)));
        assert_error(@()modified_gram_schmidt(ones(3,3,2)));
    end
end

