% test the Navigation abstract superclass

function tests = NavigationTest
    tests = functiontests(localfunctions);
end


function map_test(tc)
    map = zeros(10,10);
    map(2,3) = 1;
    
    nav = Bug2(map);  % we can't instantiate Navigation because it's abstract
    
    %% test isoccupied method
    % row vector
    tc.verifyTrue( nav.isoccupied([3,2]) );
    tc.verifyFalse( nav.isoccupied([3,3]) );
    
    % column vector
    tc.verifyTrue( nav.isoccupied([3,2]') );
    tc.verifyFalse( nav.isoccupied([3,3]') );
    
    % separate args
    tc.verifyTrue( nav.isoccupied(3,2) );
    tc.verifyFalse( nav.isoccupied(3,3) );
    
    % out of bound
    tc.verifyTrue( nav.isoccupied([20 20]) );
    
    % multiple points
    tc.verifyEqual( nav.isoccupied([3 2; 20 20; 3 3]'), [true true false] );
    
    tc.verifyEqual( nav.isoccupied([3 20 3], [2 20 3]), [true true false] );
    tc.verifyEqual( nav.isoccupied([3 20 3]', [2 20 3]), [true true false] );
    tc.verifyEqual( nav.isoccupied([3 20 3], [2 20 3]'), [true true false] );
    tc.verifyEqual( nav.isoccupied([3 20 3]', [2 20 3]'), [true true false] );
    
    %% test inflation option
    nav = Bug2(map, 'inflate', 1);
    tc.verifyTrue( nav.isoccupied([3,2]) );
    tc.verifyTrue( nav.isoccupied([3,3]) );
    tc.verifyFalse( nav.isoccupied([3,4]) );
end

function maptype_test(tc)
    
    %% logical map
    map = zeros(10,10, 'logical');
    map(2,3) = true;
    
    nav = Bug2(map);  % we can't instantiate Navigation because it's abstract
    
    tc.verifyTrue( nav.isoccupied([3,2]) );
    tc.verifyFalse( nav.isoccupied([3,3]) );
    
    %% uint8 map
    map = zeros(10,10, 'uint8');
    map(2,3) = 1;
    
    nav = Bug2(map);  % we can't instantiate Navigation because it's abstract
    
    tc.verifyTrue( nav.isoccupied([3,2]) );
    tc.verifyFalse( nav.isoccupied([3,3]) );
end

function rand_test(tc)
    
    nav = Bug2();  % we can't instantiate Navigation because it's abstract
    
    %% test random number generator
    r = nav.randn;
    tc.verifySize(r, [1 1]);
    tc.verifyInstanceOf(r, 'double');
    r = nav.randn(2,2);
    tc.verifySize(r, [2 2]);
end

function randi_test(tc)
    
    nav = Bug2();  % we can't instantiate Navigation because it's abstract
    
    %% test integer random number generator
    r = nav.randi(10);
    tc.verifySize(r, [1 1]);
    tc.verifyEqual(r, floor(r)); % is it an integer value
    tc.verifyInstanceOf(r, 'double');
    r = nav.randi(10, 2,2);
    tc.verifySize(r, [2 2]);
    tc.verifyEqual(r, floor(r));
    
    % check range
    r = nav.randi(10, 100,1);
    tc.verifyTrue(min(r) >= 1);
    tc.verifyTrue(max(r) <= 10);
end