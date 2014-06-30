using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Hexagonal;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Base;

namespace WingmanPathPlanning.Planner
{
    class IterativeBackPropEnhPathPlanner: PathPlanner 
    {
        public IterativeBackPropEnhPathPlanner(HexagonalMap map, Robot agent)
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
            PlanningNode startNode = graph[level].mNodes[currentNodeIdx];
            subpath.AddPos(startNode.pos);
            _agent.Update(subpath, localEntropy);
            int endLevel = graph.planningLength - 1;
            int nodeNum;
            int edgeNum;

            double[][] estimatedReward = new double[endLevel - level + 1][];
            double[][] futureReward = new double[endLevel - level + 1][];
            double[][] instantReward = new double[endLevel - level + 1][];
            for (int l = level; l <= endLevel; l++)
            {
                nodeNum = graph[l].mNodes.Count;

                estimatedReward[l-level] = new double[nodeNum];
                futureReward[l-level] = new double[nodeNum];
                instantReward[l-level] = new double[nodeNum];

                for (int i = 0; i < nodeNum; i++)
                {
                    PlanningNode node = graph[l].mNodes[i];
                    instantReward[l-level][i] = GetEstimation(_agent, localEntropy, node.pos, _map);
                }
            }

            nodeNum = graph[endLevel].mNodes.Count;
            for (int i = 0; i < nodeNum; i++)
            {
                estimatedReward[endLevel - level][i] = instantReward[endLevel - level][i];
            }

            for (int l = endLevel - 1; l >= level; l--)
            {
                nodeNum = graph[l].mNodes.Count;
                edgeNum = graph[l].mEdges.Count;

                for (int i = 0; i < nodeNum; i++)
                {
                    PlanningNode node = graph[l].mNodes[i];
                    List<PlanningEdge> edges = graph[l].GetEdges(node);
                    List<PlanningEdge>.Enumerator e = edges.GetEnumerator();
                    while (e.MoveNext())
                    {
                        int j = graph[l + 1].GetIndex(e.Current.to);
                        if (estimatedReward[l - level + 1][j] > futureReward[l - level][i])
                        {
                            futureReward[l - level][i] = estimatedReward[l - level + 1][j];
                        }
                    }
                }

                for (int i = 0; i < nodeNum; i++)
                {
                    estimatedReward[l - level][i] = instantReward[l - level][i] + futureReward[l - level][i];
                }
            }

            // refine and return the best estimatedReward[level+1][]
            double maxActualReward = 0.0;
            //int maxIdx = GetMaxIdx(futureReward[0]);
            //double maxExpReward = futureReward[0][maxIdx];

            //while (maxActualReward < maxExpReward)
            {
                HexaPath localMaxPath = GetMaxPath(level, startNode, graph, estimatedReward);
                maxActualReward = _agent.Score(localMaxPath, localEntropy);
                //futureReward[0][maxIdx] = maxActualReward;

                //maxIdx = GetMaxIdx(futureReward[0]);
                //maxExpReward = futureReward[0][maxIdx];
            }

            return maxActualReward; // futureReward[0][currentNodeIdx];
        }

        HexaPath GetMaxPath(int level, PlanningNode startNode, PathPlanningGraph graph, double[][] estimatedReward)
        {
            HexaPath path = new HexaPath();
            int endLevel = graph.planningLength-1;
            //path.AddPos(startNode.pos);
            HexaPos lastPos = startNode.pos;

            for (int t = level+1; t <= endLevel; t++)
            {
                int index = FindMax(lastPos, estimatedReward[t-level], t, graph);
                path.AddPos(graph[t].mNodes[index].pos);
                lastPos = path[path.Length-1];
            }

            return path;
        }

        int GetMaxIdx(double[] array)
        {
            double max = -0.1;
            int maxIdx = 0;
            for (int i = 0; i < array.Length; i++)
            {
                if (array[i] > max)
                {
                    max = array[i];
                    maxIdx = i;
                }
            }

            return maxIdx;
        }
    }
}
