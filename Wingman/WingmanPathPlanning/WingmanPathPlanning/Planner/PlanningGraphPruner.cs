using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Hexagonal;

namespace WingmanPathPlanning.Planner
{
    class PlanningGraphPruner
    {
        public PathPlanningGraph GetPlanningGraph(PathPlanningGraph graph, HexaPos start)
        {
            PathPlanningGraph newGraph = new PathPlanningGraph(graph);

            if (newGraph.planningLength > 0)
            {
                for (int i = newGraph[0].mNodes.Count -1 ; i >=0 ; i--)
                {
                    if (newGraph[0].mNodes[i].pos.X != start.X 
                        ||
                        newGraph[0].mNodes[i].pos.Y != start.Y)
                    {
                        newGraph.RemovePlanningNode(newGraph[0].mNodes[i], 0);
                    }
                }
            }

            return newGraph;
        }

        public PathPlanningGraph GetPlanningGraph(PathPlanningGraph graph, HexaPos start, HexaPos end)
        {
            PathPlanningGraph newGraph = GetPlanningGraph(graph, start);

            if (newGraph.planningLength > 0)
            {
                for (int i = newGraph[newGraph.planningLength-1].mNodes.Count - 1; i >= 0; i--)
                {
                    if (newGraph[newGraph.planningLength-1].mNodes[i].pos.X != end.X
                        ||
                        newGraph[newGraph.planningLength-1].mNodes[i].pos.Y != end.Y)
                    {
                        newGraph.RemovePlanningNode(newGraph[newGraph.planningLength - 1].mNodes[i], newGraph.planningLength-1);
                    }
                }
            }


            return newGraph;
        }

        public PathPlanningGraph ForwardPrune(PathPlanningGraph graph)
        {
            PathPlanningGraph newGraph = new PathPlanningGraph(graph);

            for (int t = 0; t <= newGraph.planningLength - 1; t++)
            {
                for (int i = newGraph[t].mNodes.Count - 1; i >= 0; i--)
                {
                    if (t == newGraph.planningLength - 1)
                    {
                        if (!newGraph.hasIn(newGraph[t].mNodes[i], t))
                        {
                            newGraph.RemovePlanningNode(newGraph[t].mNodes[i], t);
                        }
                    }
                    else if (t == 0)
                    {
                        if (!newGraph.hasOut(newGraph[t].mNodes[i], t))
                        {
                            newGraph.RemovePlanningNode(newGraph[t].mNodes[i], t);
                        }
                    }
                    else
                    {
                        if (!newGraph.hasIn(newGraph[t].mNodes[i], t)
                            ||
                            !newGraph.hasOut(newGraph[t].mNodes[i], t))
                        {
                            newGraph.RemovePlanningNode(newGraph[t].mNodes[i], t);
                        }
                    }
                }
            }

            return newGraph;
        }

        public PathPlanningGraph BackwardPrune(PathPlanningGraph graph)
        {
            PathPlanningGraph newGraph = new PathPlanningGraph(graph);

            for (int t = newGraph.planningLength - 1; t >= 0; t--)
            {
                for (int i = newGraph[t].mNodes.Count - 1; i >= 0; i--)
                {
                    if(t==newGraph.planningLength - 1)
                    {
                        if(!newGraph.hasIn(newGraph[t].mNodes[i], t))
                        {
                            newGraph.RemovePlanningNode(newGraph[t].mNodes[i], t);
                        }
                    }
                    else if(t==0)
                    {
                        if(!newGraph.hasOut(newGraph[t].mNodes[i], t))
                        {
                            newGraph.RemovePlanningNode(newGraph[t].mNodes[i], t);
                        }
                    }
                    else
                    {
                        if (!newGraph.hasIn(newGraph[t].mNodes[i], t)
                            ||
                            !newGraph.hasOut(newGraph[t].mNodes[i], t))
                        {
                            newGraph.RemovePlanningNode(newGraph[t].mNodes[i], t);
                        }
                    }

                }
            }

            return newGraph;
        }
    }
}
