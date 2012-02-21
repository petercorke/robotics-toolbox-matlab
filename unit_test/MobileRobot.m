%% This is for testing the SerialLink models in the robotics Toolbox
function test_suite = TestRobotToolboxSerialLinkModels
  initTestSuite;
%% Mobile robot
%    Map                        - point feature map object
function Test_Map
    map = Map(20,10);
%    RandomPath                 - driver for Vehicle object
function Test_Random_Path
    V = diag([0.005, 0.5*pi/180].^2);
    veh = Vehicle(V);
    veh.add_driver(RandomPath(10));
    
%    RangeBearingSensor         - "laser scanner" object
function Test_RangeBearingSensor
    V = diag([0.005, 0.5*pi/180].^2);
    W = diag([0.1, 1*pi/180].^2);
    veh = Vehicle(V);
    map = Map(20,10);
    sensor = RangeBearingSensor(veh,map,W);
  

%    Vehicle                    - construct a mobile robot object
function Test_Vehicle
    V = diag([0.005, 0.5*pi/180].^2);
    veh = Vehicle(V);
%
%%     Localization
%        EKF                    - extended Kalman filter object
function Test_EKF
    V = diag([0.005, 0.5*pi/180].^2);
    veh = Vehicle(V);
    po = diag([0.005 0.005 0.001].^2);
    ekf = EKF(veh,V,po);
%        ParticleFilter         - Monte Carlo estimator
function Test_ParticleFilter
    V = diag([0.005, 0.5*pi/180].^2);
    W = diag([0.1, 1*pi/180].^2);
    Q = diag([0.1, 0.1, 1*pi/180].^2);
    L = diag([0.1, 0.1]);
    veh = Vehicle(V);
    map = Map(20,10);
    sensor = RangeBearingSensor(veh,map,W);
    pf = ParticleFilter(veh,sensor,Q,L,1000);
%
%%     Path planning
%        Bug2                   - bug navigation
function Test_Bug2
    load map1;
    bug = Bug2(map);
    bug.goal = [50; 35];
    bug.path([20; 10]);
%        DXform                 - distance transform from map
function Test_DXform
    % run DXform with map1
    load map1;
    dx = DXform(map);
    
%        Dstar                  - D* planner
function Test_Dstar
    load map1;
    ds = Dstar(map);

%        PRM                    - probabilistic roadmap planner
function Test_PRM
    load map1;
    prm = PRM(map);
%        RRT                    - rapidly exploring random tree
function Test_RRT
    load map1;
    rrt = RRT();
