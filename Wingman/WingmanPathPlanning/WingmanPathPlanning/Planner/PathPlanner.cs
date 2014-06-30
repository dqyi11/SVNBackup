using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Hexagonal;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Base;

namespace WingmanPathPlanning.Planner
{
    public abstract class PathPlanner
    {
        protected HexagonalMap _map;
        protected Robot _agent;
        public double[,] _localEntropy;

        public PathPlanner(HexagonalMap map, Robot agent)
        {
            _map = map;
            _agent = agent;
            _localEntropy = _map.GetMapStateMgr().CopyEntropy();
        }

        public double GetEstimation(Agent agent, double[,] entropy, HexaPos pos, HexagonalMap map)
        {
            double estimation = entropy[pos.X, pos.Y];
            List<HexaPos> neighbors = map.GetHexes(pos.X, pos.Y, agent.GetObservationRange());
            List<HexaPos>.Enumerator e = neighbors.GetEnumerator();
            while (e.MoveNext())
            {
                estimation += agent.confidenceFactor * entropy[e.Current.X, e.Current.Y];
            }
            return estimation;
            /*
            double[,] localEntropy = (double[,])entropy.Clone();
            HexaPath localPath = new HexaPath();
            localPath.AddPos(pos);
            return agent.Score(localPath, localEntropy);
             */
        }

        public double ScorePath(Agent agent, double[,] entropy, HexaPath path)
        {
            double score = 0.0;
            double[,] localEntropy = (double[,])entropy.Clone();

            score = agent.Score(path, localEntropy);

            return score;
        }

        public abstract HexaPath FindPath(PathPlanningGraph graph, HexaPos start);
    }
}
