% run the tests for Travis CI
%
% Travis file system looks like this:
%    ./           ** RTB is unpacked at this level, not its own folder
%    ./unit_test  ** WORKING folder
%	 ./lib/toolbox-common
%    ./lib/spatial-math-toolbox

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

% for other toolboxes
addpath ../lib/toolbox-common-matlab
addpath ../lib/spatial-math

% for RTB
addpath ..
addpath ../models
addpath ../data
addpath ../simulink

%path
system('mount');

originalDir = pwd

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

%% Run all unit tests in my repository
fprintf('---------------------------------- Run the unit tests ------------------------------------\n')

results = runner.run(suite);

%% Assert no tests failed
assert(all(~[results.Failed]));
