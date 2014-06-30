using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using PathPlanner.Hexagonal;

namespace PathPlanner.Planner
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


        public PathPlanningGraph GetPathPlanningGraph(HexaPos startPos, int planningLength)
        {
            PathPlanningGraph planningGraph = new PathPlanningGraph(planningLength);
            List<HexaPos> currentSet = new List<HexaPos>();
            List<HexaPos> nextSet = new List<HexaPos>();
            currentSet.Add(startPos);

            PlanningNode start_node = new PlanningNode(startPos);
            planningGraph.AddPlanningNode(start_node, 0);

            // create vertex
            for (int t = 1; t < planningLength; t++)
            {
                List<HexaPos>.Enumerator e1 = currentSet.GetEnumerator();
                nextSet.Clear();
                while(e1.MoveNext())
                {
                    HexaPos currentPos = e1.Current;

                    List<HexaPos> hexes = _topologicalGraph.GetMap().GetHexes(currentPos.X, currentPos.Y, 1, true);

                    List<HexaPos>.Enumerator e = hexes.GetEnumerator();
                    while (e.MoveNext())
                    {
                        Hex currentHex = _topologicalGraph.GetMap().GetHex(e.Current.X, e.Current.Y);
                        if (false == _topologicalGraph.GetMap().MapState.IsObstacle(currentHex))
                        {
                            if(false == planningGraph[t].hasNode(e.Current.X, e.Current.Y))
                            {
                                PlanningNode node = new PlanningNode(e.Current);
                                planningGraph.AddPlanningNode(node, t);
                            
                                nextSet.Add(new HexaPos(e.Current.X, e.Current.Y));
                            }
                        }
                    }
                }
                currentSet.Clear();
                foreach(HexaPos pos in nextSet)
                {
                    currentSet.Add(pos);
                }
  
            }

         
            // create edge
            for (int t = 0; t < planningLength - 1; t++)
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
