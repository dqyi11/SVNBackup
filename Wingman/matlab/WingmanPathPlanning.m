
global STAY EAST SE SW WEST NW NE
global N LimitedStep ExpectedStep;
global HexagonSize;
global P h;
global obstacle;

STAY = 0;
EAST = 1;
SE = 2;
SW = 3;
WEST = 4;
NW = 5;
NE = 6;

N = 16;
HexagonSize = 1;
LimitedStep = 6;
ExpectedStep = 3;
tolerantDistance = 2;

%init agent position
agent = [5,5];
%actionSequence = [EAST,EAST,EAST,EAST,EAST];
actionSequence = [EAST,EAST,EAST,EAST,NE,NE,NE];
%actionSequence = [SW,SW,EAST,EAST,EAST,EAST,EAST,EAST,NW,NE,NW];
%actionSequence = [SW,SW,EAST,EAST];
%actionSequence = [SW,SW,EAST,EAST,EAST,EAST,EAST,EAST,NW,NE,NW,NE,WEST,WEST,...
%    WEST,WEST,WEST,WEST,NE,NW,NE,NW,NE,EAST,EAST,EAST,EAST];


InitMap();

AddObstacle(7,7);
AddObstacle(6,8);
AddObstacle(7,9);
%AddObstacle(4,6);
%AddObstacle(11,7);

UpdateMapColor();

%Move the robot in Map
[agent, agentPath] = GeneratePathInMap(agent, actionSequence);
UpdateMap(agentPath);

startNode = agentPath(1,:);

%Plan the robot
%robotPath = PlanRobotPath(startNode, agentPath, tolerantDistance);
%P = UpdateMap(robotPath);

%robotPath2 = PlanRobotPathWithDFS(startNode, agentPath, tolerantDistance);

robotPath3 = PlanRobotPathWithGA(startNode, agentPath, tolerantDistance);