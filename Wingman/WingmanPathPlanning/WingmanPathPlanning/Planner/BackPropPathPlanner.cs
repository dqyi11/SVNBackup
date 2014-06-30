using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Hexagonal;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Base;

namespace WingmanPathPlanning.Planner
{
    class BackPropPathPlanner : PathPlanner 
    {
        public BackPropPathPlanner(HexagonalMap map, Robot agent)
            : base(map, agent)
        {
        }

        public override HexaPath FindPath(PathPlanningGraph graph, HexaPos start)
        {
            int planningLength = graph.planningLength;
            HexaPath path = new HexaPath();
            path.AddPos(start);
            _agent.Update(path, _localEntropy);

            double[][] estimations = Backpropagation(1, graph, _localEntropy);
            for(int i=1;i<planningLength;i++)
            {
                Console.WriteLine("At level " + i.ToString());
                for(int j=0;j<estimations[i].Length;j++)
                {
                    int posX = graph[i].mNodes[j].pos.X;
                    int posY = graph[i].mNodes[j].pos.Y;
                    Console.WriteLine("Pos["+posX.ToString()+","+posY.ToString()
                        +"]="+estimations[i][j].ToString());
                }
            }

            for(int t=1;t<planningLength;t++)
            {
                //_agent.Update(path, _localEntropy);

                //double[] estimations = Backpropagation(t, graph, _localEntropy);
                int index = FindMax(path[t-1], estimations, t, graph);

                path.AddPos(graph[t].mNodes[index].pos);
                _agent.Update(path, _localEntropy);
            }

            return path;
        }

        int FindMax(HexaPos currentPos, double [][] values, int level, PathPlanningGraph graph)
        {
            double refVal = -0.1;
            int maxIdx = 0;

            PlanningNode currentNode = graph[level - 1].GetNode(currentPos);
            List<PlanningEdge> edges = graph[level - 1].GetEdges(currentNode);

            List<PlanningEdge>.Enumerator e = edges.GetEnumerator();

            while (e.MoveNext())
            {
                int nextIdx = graph[level].GetIndex(e.Current.to);
                if (values[level][nextIdx] > refVal)
                {
                    maxIdx = nextIdx;
                    refVal = values[level][nextIdx];
                }
            }

            return maxIdx;
        }

        double[][] Backpropagation(int level, PathPlanningGraph graph, double[,] entropy)
        {
            double[,] localEntropy = (double[,])entropy.Clone();
            int endLevel = graph.planningLength - 1;

            double[][] estimatedReward = new double[graph.planningLength][];
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
                        if (estimatedReward[l+1][j] > futureReward[i])
                        {
                            futureReward[i] = estimatedReward[l+1][j];
                        }
                    }
                }

                estimatedReward[l] = new double[nodeNum];
                for (int i = 0; i < nodeNum; i++)
                {
                    estimatedReward[l][i] = instantReward[i] + futureReward[i];
                }   
            }
            return estimatedReward;
        }
    }
}
