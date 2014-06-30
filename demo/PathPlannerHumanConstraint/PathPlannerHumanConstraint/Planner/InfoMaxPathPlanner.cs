using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using PathPlanner.Hexagonal;

namespace PathPlanner.Planner
{
    class InfoMaxPathPlanner : PathPlanner
    {
        public bool iteratingOnce = true;

        public InfoMaxPathPlanner(HexagonalMap map, Robot agent)
            : base(map, agent)
        {
        }

        public HexaPath FindPath(PathPlanningGraph graph, HexaPos start)
        {
            HexaPath path = null;
            double currentScore = 0.0;

            PlanningNode startNode = graph[0].GetNode(start);
            ExpandingNode root = new ExpandingNode(startNode);
            ExpandingTree expandingTree = new ExpandingTree(root);

            Console.WriteLine("The number of complete expanding node is " + graph.GetExpandingNodeNumber());

            bool exhaustivelyEnumerated = false;
            bool stopCritera = false;
            int counter = 0;

            HexaPath maxPath = null;
            double maxScore = 0.0;
        
            do
            {
                path = ExpandToFindPath(expandingTree, graph, _localEntropy);

                if (path == null)
                {
                    stopCritera = true;
                }
                else
                {
                    currentScore = ScorePath(_agent, _localEntropy, path);
                    if (currentScore > maxScore)
                    {
                        maxScore = currentScore;
                        maxPath = path;
                    }
                }

                //expandingTree.Draw("Expanding-Tree-" + counter.ToString());
                counter++;
                Console.WriteLine(counter + ", " + maxScore + ", " + expandingTree.nodeNum);
            }
            while( (iteratingOnce == false || exhaustivelyEnumerated == true) && (stopCritera==false) );

            //expandingTree.Draw("Expanding-Tree-N");
            Console.WriteLine("The number of node expanded is " + expandingTree.nodeNum);
           
            return maxPath;
        }

        HexaPath ExpandToFindPath(ExpandingTree tree, PathPlanningGraph graph, double[,] entropy)
        {
            HexaPath path = null;
            int stopLevel = graph.planningLength - 1;
            double[,] localEntropy = (double[,])entropy.Clone();

            ExpandingNode start = tree.GetMaxLeafNode(stopLevel);
            if (start == null)
            {
                return path;
            }

            ExpandingNode expandingNode = start;

            // Get subpath
            path = tree.GetPath(start);

            UpdateNodeReward(start, path, localEntropy, graph);

            // apply path
            //_agent.Update(path, localEntropy);

            // Expand node till reaching end level
            for (int cl = path.Length; cl <= stopLevel; cl++) 
            {
                expandingNode = NodeSpanning(tree, expandingNode, path, entropy, graph, _map);
                path.AddPos(expandingNode.planningNode.pos);
            }

            // score the path and back propagate minVal
            double currentScore = ScorePath(_agent, entropy, path);

            expandingNode.maxVal = currentScore;
            tree.BackPropagateMinVal(expandingNode, currentScore);

            tree.Freeze(currentScore);

            return path;
        }

        HexaPath GetMaxPath(ExpandingTree tree)
        {
            HexaPath path = new HexaPath();
            ExpandingNode root = tree.GetRoot();
            path.AddPos(root.planningNode.pos);

            ExpandingNode nextChild = tree.GetMaxChild(root);
            while (nextChild != null)
            {
                path.AddPos(nextChild.planningNode.pos);
                nextChild = tree.GetMaxChild(nextChild);
            }

            return path;
        }

        ExpandingNode NodeSpanning(ExpandingTree tree, ExpandingNode node, HexaPath path, double[,] entropy, PathPlanningGraph graph, HexagonalMap map)
        {
            PlanningNode planNode = node.planningNode;
            List<ExpandingNode> newGenerated = new List<ExpandingNode>();
            // find all child nodes
            int curLevel = path.Length - 1;
            if (curLevel < graph.planningLength - 1)
            {
                List<PlanningEdge> nextEdges = graph[curLevel].GetEdges(planNode);
                List<PlanningEdge>.Enumerator enumEd = nextEdges.GetEnumerator();
                while (enumEd.MoveNext())
                {
                    ExpandingNode newNode = new ExpandingNode(enumEd.Current.to);
                    tree.AddToParent(newNode, node);
                    newGenerated.Add(newNode);

                    // if new node is already end level, 
                    // set it as EXPANDED
                    if (curLevel == graph.planningLength - 2)
                    {
                        newNode.state = ExpandingNode.STATE.EXPANDED;
                    }
                }
            }

            // set node to EXPANDED
            node.state = ExpandingNode.STATE.EXPANDED;

            //update the new generated node
            List<ExpandingNode>.Enumerator e2 = newGenerated.GetEnumerator();
            while (e2.MoveNext())
            {
                HexaPath tempPath = tree.GetPath(e2.Current);
                double[,] tempEntropy = (double[,])entropy.Clone();
                UpdateNodeReward(e2.Current, tempPath, tempEntropy, graph);
            }

            //find max node
            double maxNodeVal = 0.0;
            ExpandingNode maxNode = null;
            List<ExpandingNode>.Enumerator e3 = newGenerated.GetEnumerator();
            while (e3.MoveNext())
            {
                if (e3.Current.maxVal > maxNodeVal)
                {
                    maxNode = e3.Current;
                    maxNodeVal = e3.Current.maxVal;
                }
                else
                {
                    if (maxNode == null)
                    {
                        maxNode = e3.Current;
                    }
                }
            }

            return maxNode;
        }

        void UpdateNodeReward(ExpandingNode node, HexaPath path, double[,] entropy, PathPlanningGraph graph)
        {
            PlanningNode planNode = node.planningNode;
            double[,] localEntropy = (double[,])entropy.Clone();
            node.instRwd = GetInstantReward(path, localEntropy, graph);
            node.futrRwd = GetEstimatedMaxFutureReward(planNode, path, localEntropy, graph);
            // update max val
            node.maxVal = node.instRwd + node.futrRwd;
        }

        double GetInstantReward(HexaPath path, double[,] entropy, PathPlanningGraph graph)
        {
            double instantReward = 0.0;
            // apply path to local entropy and calc instant reward
            for (int t = 0; t < path.Length; t++)
            {
                instantReward += GetEstimation(_agent, entropy, path[t], _map);
                HexaPath tempPath = new HexaPath();
                tempPath.AddPos(path[t]);

                // update entropy
                _agent.Update(tempPath, entropy);
            }
            return instantReward;
        }

        double GetEstimatedMaxFutureReward(PlanningNode node, HexaPath path, double[,] entropy, PathPlanningGraph graph)
        {
            int endLevel = graph.planningLength - 1;
            int currentLevel = path.Length - 1;

            if (endLevel == currentLevel)
            {
                return 0.0;
            }

            double maxFutureScore = 0.0;

            // backtrack
            
            //start from end level, init future score as 0
            int nodeNum = 0;
            double[] futureScore = null;
            double[] instantScore = null;
            double[] totalScore = null;

            for (int l = endLevel; l > currentLevel; l--)
            {
                nodeNum = graph[l].mNodes.Count;
                futureScore = new double[nodeNum];
                instantScore = new double[nodeNum];

                for (int i = 0; i < nodeNum; i++)
                {
                    PlanningNode tempNode = graph[l].mNodes[i];
                    instantScore[i] = GetEstimation(_agent, entropy, tempNode.pos, _map);

                    if (l < endLevel)
                    {
                        List<PlanningEdge> edges = graph[l].GetEdges(tempNode);
                        List<PlanningEdge>.Enumerator e = edges.GetEnumerator();
                        while (e.MoveNext())
                        {
                            int j = graph[l + 1].GetIndex(e.Current.to);
                            if (totalScore[j] > futureScore[i])
                            {
                                futureScore[i] = totalScore[j];
                            }
                        }
                    }
                    else
                    {
                        futureScore[i] = 0.0;
                    }
                }

                totalScore = new double[nodeNum];
                for (int i = 0; i < nodeNum; i++)
                {
                    totalScore[i] = instantScore[i] + futureScore[i];
                }
            }

            // estimate future reward
            HexaPos currentPos = node.pos;
            List<PlanningEdge> nextEdges = graph[currentLevel].GetEdges(node);
            List<PlanningEdge>.Enumerator enumEdge = nextEdges.GetEnumerator();
            while (enumEdge.MoveNext())
            {
                int j = graph[currentLevel + 1].GetIndex(enumEdge.Current.to);
                if (totalScore[j] > maxFutureScore)
                {
                    maxFutureScore = totalScore[j];
                }
            }

            return maxFutureScore;
        }

        ExpandingTree GetExclusiveExpandingTree(PathPlanningGraph graph, HexaPos start)
        {
            PlanningNode startNode = graph[0].GetNode(start);
            ExpandingNode root = new ExpandingNode(startNode);
            ExpandingTree expandingTree = new ExpandingTree(root);
            bool quit = false;

            for (int l = 0; l < graph.planningLength-1; l++)
            {
                while (expandingTree.GetNewNodeCountByLevel(l)>0)
                {
                    quit = false;
                    for(int i=0;i<expandingTree.nodeList.Count && quit==false;i++)
                    {
                        ExpandingNode currentNode = expandingTree.nodeList[i];
                        if (currentNode.level == l && currentNode.state == ExpandingNode.STATE.NEW)
                        {
                            PlanningNode planNode = currentNode.planningNode;

                            List<PlanningEdge> edges = graph[l].GetEdges(planNode);
                            List<PlanningEdge>.Enumerator e2 = edges.GetEnumerator();
                            while (e2.MoveNext())
                            {
                                ExpandingNode newNode = new ExpandingNode(e2.Current.to);
                                expandingTree.AddToParent(newNode, currentNode);

                                if (l == graph.planningLength - 2)
                                {
                                    newNode.state = ExpandingNode.STATE.EXPANDED;
                                }
                            }

                            currentNode.state = ExpandingNode.STATE.EXPANDED;

                            quit = true;
                        }
                    }
                }

            }

            return expandingTree;
        }

        public int GetExclusiveExpandingNodeNum(PathPlanningGraph graph, HexaPos start)
        {
            ExpandingTree tree = GetExclusiveExpandingTree(graph, start);

            tree.Draw("EXCLUSIVE-EXPANDING-TREE");

            return tree.nodeNum;
        }
    }
}
