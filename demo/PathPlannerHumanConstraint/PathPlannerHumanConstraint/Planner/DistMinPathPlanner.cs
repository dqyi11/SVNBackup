using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using QuickGraph;
using QuickGraph.Graphviz;
using PathPlanner.Hexagonal;
using EMK.Cartography;

namespace PathPlanner.Planner
{
    class DistMinPathPlanner : PathPlanner
    {
        public DistMinPathPlanner(HexagonalMap map, Robot agent)
            : base(map, agent)
        {
        }

        public HexaPath FindPath(TopologyGraph graph, HexaPos start, HexaPos end)
        {
            HexaPath path = new HexaPath();

           	Graph G = new Graph();

            int gridWidth = graph.hexes.GetLength(0);
            int gridHeight = graph.hexes.GetLength(1);

            List<Node> nodelist = new List<Node>();
            for (int j = 0; j < gridHeight; j++)
            {
                for (int i = 0; i < gridWidth; i++)
                {
                    HexaPos tempPos = graph.hexes[i,j].GetHexaPos();
                    Hex tempHex = _map.GetHex(tempPos.X, tempPos.Y);
                    if (_map.mapState.IsObstacle(tempHex)==false)
                    {
                        Node n = G.AddNode(tempPos.X, tempPos.Y, 0);
                        nodelist.Add(n);
                    }
                }
            }

            List<HexaGridEdge>.Enumerator etE = graph.edgeList.GetEnumerator();
            while (etE.MoveNext())
            {
                Node a = findNode(nodelist, etE.Current.GetA().X, etE.Current.GetA().Y);
                Node b = findNode(nodelist, etE.Current.GetB().X, etE.Current.GetB().Y);
                if (a != null && b != null)
                {
                    G.AddArc(a, b, 1);
                    G.AddArc(b, a, 1);
                }
            }

            Node startNode = findNode(nodelist, start.X, start.Y);
            Node endNode = findNode(nodelist, end.X, end.Y);
		
			Console.WriteLine("- Best path to reach "+startNode.ToString()+" from "+endNode.ToString()+" :");
			AStar AS = new AStar(G);
            if (AS.SearchPath(startNode, endNode))
            {
				// foreach (Arc A in AS.PathByArcs) Console.WriteLine( A.ToString() );
                foreach (Node N in AS.PathByNodes)
                {
                    Console.WriteLine(N.ToString());
                    HexaPos n = new HexaPos((int)N.X, (int)N.Y);
                    path.AddPos(n);
                }
            }
			else Console.WriteLine( "No result !" );

            return path;
        }

        Node findNode(List<Node> list, int x, int y)
        {
            List<Node>.Enumerator e = list.GetEnumerator();
            while (e.MoveNext())
            {
                if (e.Current.X == x && e.Current.Y == y)
                {
                    return e.Current;
                }
            }
            return null;
        }
    }
}
