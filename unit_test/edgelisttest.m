im = [
     0     0     0     0     0     0     0     0
     0     0     0     0     0     0     0     0
     0     0     0     1     1     1     0     0
     0     0     0     1     1     1     0     0
     0     0     1     1     1     1     0     0
     0     0     0     1     1     1     0     0
     0     0     0     0     0     0     0     0
     0     0     0     0     0     0     0     0
     0     0     0     0     0     0     0     0
     0     0     0     0     0     0     0     0
     ];
 
 seeds = [3 5; 4 6; 5 6; 4 5; 2 5; 3 6; 4 7; 2 6];
 for seed=seeds'
     edge = edgelist(im, seed);
     for e=edge
         assert( im(e(2),e(1)) == im(seed(2), seed(1)) );
     end
     
     edge = edgelist(im, seed, 1);
     for e=edge
         assert( im(e(2),e(1)) == im(seed(2), seed(1)) );
     end
 end


