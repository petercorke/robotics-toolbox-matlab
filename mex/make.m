fprintf('** building mex file\n');

pth = which('imorph.m');
pth = fileparts(pth);
cd( fullfile(pth, 'mex') );

mex frne.c ne.c vmath.c
