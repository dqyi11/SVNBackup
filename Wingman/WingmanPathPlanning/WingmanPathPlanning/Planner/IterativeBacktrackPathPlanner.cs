using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Hexagonal;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Base;

namespace WingmanPathPlanning.Planner
{
    class IterativeBacktrackPathPlanner: PathPlanner 
    {
        public IterativeBacktrackPathPlanner(HexagonalMap map, Robot agent)
            : base(map, agent)
        {
        }

        public override HexaPath FindPath(PathPlanningGraph graph, HexaPos start)
        {
            int planningLength = graph.planningLength;
            HexaPath path = new HexaPath();
            path.AddPos(start);

            return path;
        }
    }
}
