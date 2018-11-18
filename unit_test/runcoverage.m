import matlab.unittest.TestSuite;
import matlab.unittest.TestRunner;
import matlab.unittest.plugins.CodeCoveragePlugin;

% Create a TestSuite array
suite = TestSuite.fromFolder('unit_test');

% Create a runner and add the code coverage plugin
runner = TestRunner.withTextOutput;
runner.addPlugin(CodeCoveragePlugin.forFolder(pwd));

% Run the suite. This opens a code coverage report when done testing.
result = runner.run(suite)