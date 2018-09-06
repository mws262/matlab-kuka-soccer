function exceptions = test_mergeOptions()
theader('Testing mergeOptions.');

default_struct.a = 4;
default_struct.b = 'hello';
default_struct.c = [4,1];
default_struct.d.sub_a = 1;
default_struct.d.sub_b = 2;

exceptions = {};
exceptions{end+1} = do_test(@merge_with_missing_fields);


    function merge_with_missing_fields()
       tname('Fewer user options than defaults.');
       user_struct.a = 5;
       result_struct = mergeOptions(default_struct, user_struct);
       assert(result_struct.a == 5);
       assert(strcmp(result_struct.b, default_struct.b));
       assert(all(result_struct.c == default_struct.c));
       assert(result_struct.d.sub_a == 1);
       assert(result_struct.d.sub_b == 2);
    end
end

