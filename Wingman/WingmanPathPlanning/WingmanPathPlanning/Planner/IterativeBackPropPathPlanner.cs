using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Hexagonal;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Base;

namespace WingmanPathPlanning.Planner
{
    class IterativeBackPropPathPlanner : PathPlanner 
    {
        HexaPath _maxPath;
        double _maxScore;
        HexaPath _currentPath;

        public IterativeBackPropPathPlanner(HexagonalMap map, Robot agent)
            : base(map, agent)
        {
        }

        public override HexaPath FindPath(PathPlanningGraph graph, HexaPos start)
        {
            int planningLength = graph.planningLength;
            HexaPath path = new HexaPath();
            path.AddPos(start);

            for(int t=1;t<planningLength;t++)
            {
                _agent.Update(path, _localEntropy);
                _currentPath = path;
                double[] estimations = EstimateRewards(t, graph, _localEntropy);
                int index = FindMax(path[t-1], estimations, t, graph);

                path.AddPos(graph[t].mNodes[index].pos);
            }

            return path;
        }

        int FindMax(HexaPos currentPos, double [] values, int level, PathPlanningGraph graph)
        {
            double refVal = -0.1;
            int maxIdx = 0;

            PlanningNode currentNode = graph[level - 1].GetNode(currentPos);
            List<PlanningEdge> edges = graph[level - 1].GetEdges(currentNode);

            List<PlanningEdge>.Enumerator e = edges.GetEnumerator();
            while (e.MoveNext())
            {
                int nextIdx = graph[level].GetIndex(e.Current.to);
                if (values[nextIdx] > refVal)
                {
                    maxIdx = nextIdx;
                    refVal = values[nextIdx];
                }
            }

            return maxIdx;
        }

        double[] EstimateRewards(int level, PathPlanningGraph graph, double[,] entropy)
        {
            double[,] localEntropy = (double[,])entropy.Clone();
            int nodeNum = graph[level].mNodes.Count;
            double[] estimatedReward = new double[nodeNum];
            double[] futureReward = new double[nodeNum];
            double[] instantReward = new double[nodeNum];
            for (int i = 0; i < nodeNum; i++)
            {
                PlanningNode node = graph[level].mNodes[i];
                instantReward[i] = GetEstimation(_agent, localEntropy, node.pos, _map);
                futureReward[i] = Backpropagation(level, i, graph, localEntropy);
                estimatedReward[i] = instantReward[i] + futureReward[i];
            }

            return estimatedReward;
        }

        double Backpropagation(int level, int currentNodeIdx, PathPlanningGraph graph, double[,] entropy)
        {
            double[,] localEntropy = (double[,])entropy.Clone();
            HexaPath subpath = new HexaPath();
            subpath.AddPos(graph[level].mNodes[currentNodeIdx].pos);
            _agent.Update(subpath, localEntropy);
            int endLevel = graph.planningLength - 1;

            double[] estimatedReward = null;
            double[] futureReward = null;
            double[] instantReward = null;
            int nodeNum;
            int edgeNum;

            for (int l = endLevel; l >= level; l--)
            {
                nodeNum = graph[l].mNodes.Count;
                edgeNum = graph[l].mEdges.Count;
                
                instantReward = new double[nodeNum];
                futureReward = new double[nodeNum];

                for (int i = 0; i < nodeNum; i++)
                {
                    PlanningNode node = graph[l].mNodes[i];
                    instantReward[i] = GetEstimation(_agent, localEntropy, node.pos, _map);

                    List<PlanningEdge> edges = graph[l].GetEdges(node);
                    List<PlanningEdge>.Enumerator e = edges.GetEnumerator();
                    while (e.MoveNext())
                    {
                        int j = graph[l + 1].GetIndex(e.Current.to);
                        if (estimatedReward[j] > futureReward[i])
                        {
                            futureReward[i] = estimatedReward[j];
                        }
                    }
                }

                estimatedReward = new double[nodeNum];
                for (int i = 0; i < nodeNum; i++)
                {
                    estimatedReward[i] = instantReward[i] + futureReward[i];
                }
            }
            return futureReward[currentNodeIdx];
        }

        double CalcExhScore(HexaPos current, int level, PathPlanningGraph graph, double[,] entropy, HexaPath path)
        {
            _maxPath = null;
            _maxScore = 0.0;
            HexaPath newPath = (HexaPath)path.Clone();
            FindMaxPath(graph, newPath, current);
            return _maxScore;
        }

        double ScorePath(HexaPath path)
        {
            return _agent.Score(path, _localEntropy);
        }

        void FindMaxPath(PathPlanningGraph graph, HexaPath path, HexaPos newPos)
        {
            HexaPath currentPath = path.Clone();
            currentPath.AddPos(newPos);

            if (graph.planningLength == currentPath.Length)
            {
                double newScore = ScorePath(currentPath);

                if (_maxPath == null)
                {
                    _maxPath = currentPath;
                    _maxScore = newScore;
                }
                else
                {
                    if (newScore > _maxScore)
                    {
                        _maxPath = currentPath;
                        _maxScore = newScore;
                    }
                }

                return;
            }

            List<PlanningNode>.Enumerator e = graph[currentPath.Length].mNodes.GetEnumerator();
            while (e.MoveNext())
            {
                if (_map.IsAccessible(currentPath[currentPath.Length - 1], e.Current.pos) == true)
                {
                    //currentPath.AddPos(e.Current.pos);
                    FindMaxPath(graph, currentPath, e.Current.pos);
                }
            }
        }
    }
}
