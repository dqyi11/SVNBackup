using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using PathPlanner.Hexagonal;

namespace PathPlanner.Planner
{
    class TopologyGraphGenerator
    {
        HexagonalMap _map;

        public TopologyGraphGenerator(HexagonalMap map)
        {
            _map = map;
        }

        public TopologyGraph GetTopologyGraph()
        {
            TopologyGraph graph = new TopologyGraph(_map);

            return graph;
        }
    }
}
