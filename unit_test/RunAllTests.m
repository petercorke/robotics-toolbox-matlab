% run the tests for Travis CI
%
% Travis file system looks like this:
%    ./           ** RTB is unpacked at this level, not its own folder
%    ./unit_test  ** WORKING folder
%	 ./lib/common
%    ./lib/spatial-math

%% set up the test runner
import matlab.unittest.plugins.CodeCoveragePlugin
import matlab.unittest.plugins.codecoverage.CoberturaFormat
import matlab.unittest.TestRunner

suite = testsuite('IncludeSubfolders', false);
runner = TestRunner.withTextOutput;

% add a coverage report
reportFile = fullfile('..', 'coverage.xml');
reportFormat = CoberturaFormat(reportFile);
plugin = CodeCoveragePlugin.forFolder('..', 'Producing',reportFormat);
runner.addPlugin(plugin);

%% compile some codecoverage


%% setup the path
fprintf('---------------------------------- Setup path ------------------------------------\n')
fprintf('-->> current working folder is %s\n', pwd)

% for other toolboxes
addpath ../lib/common
addpath ../lib/spatial-math

% for RTB
addpath ..
addpath ../models
addpath ../data
addpath ../simulink

%path
%system('mount');

originalDir = pwd; % this is the working dir, unit_test folder

% build the Java classes
fprintf('---------------------------------- Build Java classes ------------------------------------\n')
cd ../java
system('make')
javaaddpath DHFactor.jar

% build the MEX file
fprintf('---------------------------------- Build MEX files ------------------------------------\n')
cd ../mex
make
check

cd(originalDir)
addpath ../mex

%% Run all unit tests in my repository
fprintf('---------------------------------- Run the unit tests ------------------------------------\n')

results = runner.run(suite);

% Assert no tests failed
assert(all(~[results.Failed]));

%% Build the toolbox distribution file
fprintf('---------------------------------- Build the MLTBX file ------------------------------------\n')
cd ..
% add more folders to the path to ensure they go in the MLTBX file
addpath demos
addpath examples
addpath Apps

matlab.addons.toolbox.packageToolbox('PackageToolbox.prj', 'RTB.mltbx')
