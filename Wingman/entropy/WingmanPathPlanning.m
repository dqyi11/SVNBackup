
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

%init agent position
agent = [5,5];
%actionSequence = [EAST,EAST,EAST,EAST,EAST];
actionSequence = [EAST,EAST,EAST,EAST,NE,NE,NE];
%actionSequence = [SW,SW,EAST,EAST,EAST,EAST,EAST,EAST,NW,NE,NW];
%actionSequence = [SW,SW,EAST,EAST];
%actionSequence = [SW,SW,EAST,EAST,EAST,EAST,EAST,EAST,NW,NE,NW,NE,WEST,WEST,...
%    WEST,WEST,WEST,WEST,NE,NW,NE,NW,NE,EAST,EAST,EAST,EAST];

InitMap();

UpdateMapColor();

%Move the robot in Map
[agent, agentPath] = GeneratePathInMap(agent, actionSequence);
UpdateMap(agentPath);
