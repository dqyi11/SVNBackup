using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Hexagonal;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Base;

namespace WingmanPathPlanning.Planner
{
    class TeleportPathPlanner : PathPlanner
    {
        public TeleportPathPlanner(HexagonalMap map, Robot agent)
            : base(map, agent)
        {
        }

        public override HexaPath FindPath(PathPlanningGraph graph, HexaPos start)
        {
            HexaPath path = new HexaPath();

            int planningLength = graph.planningLength;

            path.AddPos(start);

            for (int t = 1; t < planningLength; t++)
            {
                _agent.Update(path, _localEntropy);
                HexaPos onePos = GetMaxPos(t, graph);
                path.AddPos(onePos);            
            }

            return path;
        }

        HexaPos GetMaxPos(int level, PathPlanningGraph graph)
        {
            List<PlanningEdge>.Enumerator e = graph[level - 1].mEdges.GetEnumerator();
            double maxScore = -0.1;
            HexaPos maxPos = null;
            while (e.MoveNext())
            {
                double newScore = GetEstimation(_agent, _localEntropy, e.Current.to.pos, _map);
                if (newScore > maxScore)
                {
                    maxScore = newScore;
                    maxPos = e.Current.to.pos;
                }                
            }

            return maxPos;
        }

    }
}
