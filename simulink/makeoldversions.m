files = dir('*.slx');
oldversion = 'R2015a';
status = mkdir(oldversion);
for file = files'
    [~,sysname,~] = fileparts(file.name);
    fprintf('Processing %s\n', sysname);
    
    open(file.name); % open the model
    % save it 
    save_system(sysname, fullfile(oldversion, file.name), 'ExportToVersion', oldversion);
    bdclose
end