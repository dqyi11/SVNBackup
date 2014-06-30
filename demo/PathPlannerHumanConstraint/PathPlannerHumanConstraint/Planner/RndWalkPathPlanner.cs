using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using PathPlanner.Hexagonal;

namespace PathPlanner.Planner
{
    class RndWalkPathPlanner : PathPlanner
    {
        public RndWalkPathPlanner(HexagonalMap map, Robot agent)
            : base(map, agent)
        {
        }

        public HexaPath FindPath(TopologyGraph graph, HexaPos start, int planningLength)
        {
            HexaPath path = new HexaPath();

            return path;
        }


    }
}
