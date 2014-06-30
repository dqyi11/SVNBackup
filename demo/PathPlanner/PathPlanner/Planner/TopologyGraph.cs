using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using PathPlanner.Hexagonal;
using QuickGraph;
using QuickGraph.Graphviz;
using System.Diagnostics;

namespace PathPlanner.Planner
{
    class HexaAdjacentList
    {
        HexaPos _pos;
        List<HexaPos> _adjacentList;
        public HexaAdjacentList(HexaPos pos)
        {
            _pos = pos;
            _adjacentList = new List<HexaPos>();
        }

        public HexaPos GetHexaPos()
        {
            return _pos;
        }

        public void Add(HexaPos pos)
        {
            _adjacentList.Add(pos);
        }

        public bool Has(HexaPos pos)
        {
            return _adjacentList.Contains(pos);
        }

        public void Print()
        {
            Console.Write("(" + _pos.X + " " + _pos.Y + ") : ");
            List<HexaPos>.Enumerator e = _adjacentList.GetEnumerator();
            while (e.MoveNext())
            {
                Console.Write("(" + e.Current.X + " " + e.Current.Y + ") ");
            }
            Console.WriteLine(" ");
        }
    }

    class HexaGridEdge
    {
        HexaPos _a;
        HexaPos _b;

        public HexaGridEdge(HexaPos a, HexaPos b)
        {
            _a = a;
            _b = b;
        }

        public HexaPos GetA()
        {
            return _a;
        }

        public HexaPos GetB()
        {
            return _b;
        }

        public override bool Equals(object obj)
        {
            if((this._a.Equals(((HexaGridEdge)obj)._a)
                && this._b.Equals(((HexaGridEdge)obj)._b))
                || 
                (this._b.Equals(((HexaGridEdge)obj)._a)
                && this._a.Equals(((HexaGridEdge)obj)._b)))
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        public void Print()
        {
            Console.WriteLine("Edge from (" + _a.X + " " + _a.Y + ") to (" + _b.X + " " + _b.Y + ")");
        }
    }

    class TopologyGraph
    {
        HexagonalMap _map;
        HexaAdjacentList[,] _hexes;
        List<HexaGridEdge> _edgeList;
        UndirectedGraph<string, QuickGraph.Edge<string>> _mG;

        public HexaAdjacentList[,] hexes
        {
            get { return _hexes;  }
        }

        public List<HexaGridEdge> edgeList
        {
            get { return _edgeList;  }
        }

        public TopologyGraph(HexagonalMap map)
        {
            _map = map;
            _hexes = new HexaAdjacentList[_map.mapWidth, _map.mapHeight];
            _edgeList = new List<HexaGridEdge>();
            _mG = new UndirectedGraph<string, QuickGraph.Edge<string>>();

            Init();

            InitGraph();
        }

        public UndirectedGraph<string, QuickGraph.Edge<string>> GetGraph()
        {
            return _mG;
        }

        void Init()
        {
            for (int i = 0; i < _map.mapWidth; i++)
            {
                for (int j = 0; j < _map.mapHeight; j++)
                {
                    _hexes[i, j] = new HexaAdjacentList(new HexaPos(i, j));

                    List<HexaPos> adjacent = _map.GetHexes(i, j, 1);

                    List<HexaPos>.Enumerator e = adjacent.GetEnumerator();

                    while (e.MoveNext())
                    {
                        _hexes[i, j].Add(e.Current);

                        AddEdge(_hexes[i, j].GetHexaPos(), e.Current);
                    }
                }
            }
        }

        public HexagonalMap GetMap()
        {
            return _map;
        }

        public void AddEdge(HexaPos a, HexaPos b)
        {
            HexaGridEdge edge = new HexaGridEdge(a, b);

            if (!_edgeList.Contains(edge))
            {
                _edgeList.Add(edge);
            }
        }

        public bool IsConnected(HexaPos a, HexaPos b)
        {
            return _hexes[a.X, a.Y].Has(b);
        }

        public void Print()
        {
            Console.WriteLine("Print HexaGridGraph");
            for (int i = 0; i < _map.mapWidth; i++)
            {
                for (int j = 0; j < _map.mapHeight; j++)
                {
                    _hexes[i, j].Print();
                }
            }

            Console.WriteLine(" ");

            List<HexaGridEdge>.Enumerator e = _edgeList.GetEnumerator();
            while (e.MoveNext())
            {
                e.Current.Print();
            }
        }

        private void InitGraph()
        {
            int gridWidth = _hexes.GetLength(0);
            int gridHeight = _hexes.GetLength(1);

            for (int j = 0; j < gridHeight; j++)
            {
                for (int i = 0; i < gridWidth; i++)
                {
                    _mG.AddVertex(_hexes[i,j].GetHexaPos().GetName());
                }
            }

            List<HexaGridEdge>.Enumerator etE = _edgeList.GetEnumerator();
            while (etE.MoveNext())
            {
                QuickGraph.Edge<string> e = new Edge<string>(etE.Current.GetA().GetName(),
                     etE.Current.GetB().GetName());
                _mG.AddEdge(e);
            }
        }

        public void Draw(string filename = "HexaGridGraph")
        {

            var graphviz = new GraphvizAlgorithm<string, Edge<string>>(_mG);
            string dotFile = graphviz.Generate(new FileDotEngine(), filename);
            var diagramFile = dotFile.Replace(".dot", ".png");

            string graphOutput = string.Format(@"""{0}"" -o ""{1}"" -Tpng", dotFile, diagramFile);
            Process.Start(new ProcessStartInfo("dot", graphOutput){CreateNoWindow = true, UseShellExecute = false });

        }
    }
}
