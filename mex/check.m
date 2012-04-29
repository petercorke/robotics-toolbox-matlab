fprintf('***************************************************************\n')
fprintf('************************ Puma 560 *****************************\n')
fprintf('***************************************************************\n')
clear
mdl_puma560
rdh = p560;
mdl_puma560akb
rmdh = p560m;

if exist('rne') == 3
    error('need to remove rne.mex file from @SerialLink dir');
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
