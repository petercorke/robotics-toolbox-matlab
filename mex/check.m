fprintf('***************************************************************\n')
fprintf('************************ Puma 560 *****************************\n')
fprintf('***************************************************************\n')
clear
mdl_puma560
rdh = p560;
mdl_puma560akb
rmdh = p560m;

for i=1:6
    %rdh.links(i).Jm = 0;
    %rdh.links(i).B = 0;
    %rdh.links(i).Tc = 0;
    rdh.links(i).G = abs(rdh.links(i).G);
    rmdh.links(i).Jm = 0;
    rmdh.links(i).B = 0;
    rmdh.links(i).Tc = 0;
end

check1

fprintf('\n***************************************************************\n')
fprintf('********************** Stanford arm ***************************\n')
fprintf('***************************************************************\n')

clear
mdl_stanford
stanfordm
rdh = stanf;
rdhm = stanm;

check1
