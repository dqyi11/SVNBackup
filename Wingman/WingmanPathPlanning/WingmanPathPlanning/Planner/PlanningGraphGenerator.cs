using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Hexagonal;

namespace WingmanPathPlanning.Planner
{
    class PlanningGraphGenerator
    {
        TopologyGraph _topologicalGraph;

        public PlanningGraphGenerator(TopologyGraph graph)
        {
            _topologicalGraph = graph;
        }

        public PathPlanningGraph GetPathPlanningGraph(HexaPath path, int radius)
        {
            int planningLength = path.Length;
            PathPlanningGraph planningGraph = new PathPlanningGraph(planningLength);

            // create vertex
            for (int t = 0; t < planningLength; t++)
            {
                HexaPos pivot = path[t];
                List<HexaPos> hexes = _topologicalGraph.GetMap().GetHexes(pivot.X, pivot.Y, radius, true);

                List<HexaPos>.Enumerator e = hexes.GetEnumerator();
                while (e.MoveNext())
                {
                    Hex currentHex = _topologicalGraph.GetMap().GetHex(e.Current.X, e.Current.Y);
                    if (false == _topologicalGraph.GetMap().MapState.IsObstacle(currentHex))
                    {
                        PlanningNode node = new PlanningNode(e.Current);
                        planningGraph.AddPlanningNode(node, t);
                    }
                }
            }

            // create edge
            for (int t = 0; t < planningLength-1; t++)
            {
                LevelPartite currentPartite = planningGraph[t];
                LevelPartite nextPartite = planningGraph[t + 1];

                List<PlanningNode>.Enumerator e1 = currentPartite.mNodes.GetEnumerator();
                List<PlanningNode>.Enumerator e2 = nextPartite.mNodes.GetEnumerator();

                while (e1.MoveNext())
                {
                    while (e2.MoveNext())
                    {
                        if (_topologicalGraph.IsConnected(e1.Current.pos, e2.Current.pos))
                        {
                            currentPartite.Connect(e1.Current, e2.Current);
                        }
                    }

                    e2 = nextPartite.mNodes.GetEnumerator();
                }
            }

            return planningGraph;
        }
    }
}
