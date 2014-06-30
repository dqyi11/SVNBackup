using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Hexagonal;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Base;

namespace WingmanPathPlanning.Planner
{
    class RetrackLink
    {
        public PlanningNode retrackNode;
        public int level;

        public RetrackLink(int lvl, PlanningNode node)
        {
            level = lvl;
            retrackNode = node;
        }
    }

    class RetrackLinkList
    {
        public List<RetrackLink> linkList;
        public PlanningNode mNode;

        public RetrackLinkList()
        {
            linkList = new List<RetrackLink>();
        }
    }

    class IterativeBackPropRetrackPathPlanner : PathPlanner 
    {
        RetrackLinkList [][] _retrackLinkTable;
        public IterativeBackPropRetrackPathPlanner(HexagonalMap map, Robot agent)
            : base(map, agent)
        {
        }

        public override HexaPath FindPath(PathPlanningGraph graph, HexaPos start)
        {
            init(graph);
            int planningLength = graph.planningLength;
            HexaPath path = new HexaPath();
            path.AddPos(start);

            for(int t=1;t<planningLength;t++)
            {
                _agent.Update(path, _localEntropy);

                double[] estimations = Backpropagation(t, graph, _localEntropy);
                int index = FindMax(path[t-1], estimations, t, graph);

                path.AddPos(graph[t].mNodes[index].pos);
            }

            return path;
        }

        void init(PathPlanningGraph graph)
        {
            _retrackLinkTable = new RetrackLinkList[graph.planningLength][];
            for (int t = 0; t < graph.planningLength; t++)
            {
                _retrackLinkTable[t] = new RetrackLinkList[graph[t].mNodes.Count];
                for (int i = 0; i < graph[t].mNodes.Count; i++)
                {
                    _retrackLinkTable[t][i] = new RetrackLinkList();
                    PlanningNode node = graph[t].mNodes[i];
                    _retrackLinkTable[t][i].mNode = node;
                }
            }
            // init retrack list
            for (int t1 = 0; t1 < graph.planningLength; t1++)
            {
                for (int t2 = t1 + 1; t2 < graph.planningLength; t2++)
                {
                    for (int i1 = 0; i1 < graph[t1].mNodes.Count; i1++)
                    {
                        for (int i2 = 0; i2 < graph[t2].mNodes.Count; i2++)
                        {
                            if (graph[t1].mNodes[i1].pos.X == graph[t2].mNodes[i2].pos.X
                                && graph[t1].mNodes[i1].pos.Y == graph[t2].mNodes[i2].pos.Y)
                            {
                                RetrackLink newLink = new RetrackLink(t2, graph[t2].mNodes[i2]);
                                _retrackLinkTable[t1][i1].linkList.Add(newLink);
                            }
                        }
                    }
                }
            }
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

        int FindMax(double[] values)
        {
            double refVal = -0.1;
            int maxIdx = 0;

            for (int i = 0; i < values.Length; i++)
            {
                if (values[i] > refVal)
                {
                    refVal = values[i];
                    maxIdx = i;
                }
            }
            return maxIdx;
        }


        double[] Backpropagation(int level, PathPlanningGraph graph, double[,] entropy)
        {
            double[,] localEntropy = (double[,])entropy.Clone();
            int endLevel = graph.planningLength - 1;

            double[][] estimatedReward = new double[graph.planningLength][];
            double[][] futureReward = new double[graph.planningLength][];
            double[][] instantReward = new double[graph.planningLength][];
            int nodeNum;
            int edgeNum;

            //init indepedent rewards
            double[][] independentReward = new double[graph.planningLength][];
            for(int l=0;l<graph.planningLength;l++)
            {
                independentReward[l] = new double[graph[l].mNodes.Count];
                estimatedReward[l] = new double[graph[l].mNodes.Count];
                futureReward[l] = new double[graph[l].mNodes.Count];
                instantReward[l] = new double[graph[l].mNodes.Count];
                for(int i=0;i<graph[l].mNodes.Count;i++)
                {
                    independentReward[l][i] = GetEstimation(_agent, localEntropy, graph[l].mNodes[i].pos, _map);
                    instantReward[l][i] = independentReward[l][i];
                }
            }

            bool stop = false;
            while(stop == false)
            {
                stop = true;
                for (int l = endLevel; l >= level; l--)
                {
                    nodeNum = graph[l].mNodes.Count;
                    edgeNum = graph[l].mEdges.Count;

                    for (int i = 0; i < nodeNum; i++)
                    {
                        double estRwd = 0.0;
                        double insRwd = instantReward[l][i];
                        double futRwd = 0.0;

                        PlanningNode node = graph[l].mNodes[i];

                        List<PlanningEdge> edges = graph[l].GetEdges(node);
                        List<PlanningEdge>.Enumerator e = edges.GetEnumerator();
                        while (e.MoveNext())
                        {
                            int j = graph[l + 1].GetIndex(e.Current.to);
                            if (estimatedReward[l + 1][j] > futRwd)
                            {
                                futRwd = estimatedReward[l + 1][j];
                            }
                        }

                        if (futRwd != futureReward[l][i])
                        {
                            futureReward[l][i] = futRwd;
                            stop = false;
                        }

                        estimatedReward[l][i] = instantReward[l][i] + futureReward[l][i];
                    }
   
                    // find max node and back feed
                    int maxIdx = FindMax(estimatedReward[l]);
                    for (int i = 0; i < nodeNum; i++)
                    {
                        if (i == maxIdx)
                        {
                            List<RetrackLink>.Enumerator eR = _retrackLinkTable[l][maxIdx].linkList.GetEnumerator();
                            while (eR.MoveNext())
                            {
                                int nodeLevel = eR.Current.level;
                                int nodeIdx = graph[nodeLevel].GetIndex(eR.Current.retrackNode);
                                instantReward[nodeLevel][nodeIdx] = 0;
                            }
                        }
                        else
                        {
                            List<RetrackLink>.Enumerator eR = _retrackLinkTable[l][maxIdx].linkList.GetEnumerator();
                            while (eR.MoveNext())
                            {
                                int nodeLevel = eR.Current.level;
                                int nodeIdx = graph[nodeLevel].GetIndex(eR.Current.retrackNode);
                                instantReward[nodeLevel][nodeIdx] = independentReward[nodeLevel][nodeIdx];
                            }
                        }
                    }
                }
            }
            Console.WriteLine("IBP RETRK at level " + level.ToString());
            for (int i = 0; i < estimatedReward[level].Length; i++)
            {
                int posX = graph[level].mNodes[i].pos.X;
                int posY = graph[level].mNodes[i].pos.Y;
                Console.WriteLine("Pos[" + posX.ToString() + "," + posY.ToString() + "]=" + estimatedReward[level][i].ToString());
            }
            return (double[])estimatedReward[level].Clone();
        }
    }
}
