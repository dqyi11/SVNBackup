using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Hexagonal;
using QuickGraph;
using QuickGraph.Graphviz;
using System.Diagnostics;
using DotNumerics.LinearAlgebra;

namespace WingmanPathPlanning.Map
{
    public class PlanningNode
    {
        HexaPos _pos;
        LevelPartite _parent;

        public LevelPartite parent
        {
            get
            {
                return _parent;
            }

            set
            {
                _parent = value;
            }
        }

        public PlanningNode(HexaPos pos)
        {
            _pos = pos;
        }

        public HexaPos pos
        {
            get
            {
                return _pos;
            }
        }

        public string GetName()
        {
            string name = _pos.GetName() + "@" + _parent.Level.ToString();
            return name;
        }
    }

    public class PlanningEdge
    {
        PlanningNode _from;
        PlanningNode _to;

        LevelPartite _parent;

        public LevelPartite parent
        {
            get
            {
                return _parent;
            }

            set
            {
                _parent = value;
            }
        }

        public PlanningEdge(PlanningNode from, PlanningNode to)
        {
            _from = from;
            _to = to;
        }

        public PlanningNode from
        {
            get
            {
                return _from;
            }
        }

        public PlanningNode to
        {
            get
            {
                return _to;
            }
        }
    }

    public class LevelPartite
    {
        public List<PlanningNode> mNodes;
        public List<PlanningEdge> mEdges;

        int _level;

        public LevelPartite(int level)
        {
            mNodes = new List<PlanningNode>();
            mEdges = new List<PlanningEdge>();
            _level = level;
        }

        public int Level
        {
            get
            {
                return _level;
            }

            set
            {
                _level = value;
            }
        }

        public List<PlanningEdge> GetEdges(PlanningNode node)
        {
            List<PlanningEdge> edges = new List<PlanningEdge>();
            List<PlanningEdge>.Enumerator e = mEdges.GetEnumerator();

            while (e.MoveNext())
            {
                if (e.Current.from.pos.X == node.pos.X &&
                    e.Current.from.pos.Y == node.pos.Y)
                {
                    edges.Add(e.Current);
                }
            }

            return edges;
        }

        public PlanningNode this[int index]
        {
            get
            {
                return mNodes[index];
            }
        }

        public int GetIndex(PlanningNode node)
        {
            for (int i = 0; i < mNodes.Count; i++)
            {
                if (mNodes[i] == node)
                {
                    return i;
                }
            }

            return -1;
        }

        public void AddPlanningNode(PlanningNode node)
        {
            mNodes.Add(node);
            node.parent = this;            
        }

        public void Connect(PlanningNode from, PlanningNode to)
        {
            PlanningEdge edge = new PlanningEdge(from, to);
            mEdges.Add(edge);
            edge.parent = this;
        }

        public PlanningNode GetNode(HexaPos pos)
        {
            List<PlanningNode>.Enumerator e = mNodes.GetEnumerator();
            while (e.MoveNext())
            {
                if (e.Current.pos.X == pos.X && e.Current.pos.Y == pos.Y)
                {
                    return e.Current;
                }
            }

            return null;
        }

        public List<HexaPos> GetHexes()
        {
            List<HexaPos> hexes = new List<HexaPos>();

            List<PlanningNode>.Enumerator e = mNodes.GetEnumerator();

            while (e.MoveNext())
            {
                hexes.Add(e.Current.pos);
            }

            return hexes;
        }
    }

    public class PathPlanningGraph
    {
        int _planningLength;
        LevelPartite[] _timeLevels;

        public int planningLength
        {
            get
            {
                return _planningLength;
            }
        }

        public LevelPartite this[int index]
        {
            get
            {
                return _timeLevels[index];
            }
        }

        public bool hasIn(PlanningNode node, int level)
        {
            if (level <= 0 || level >= _planningLength)
            {
                return false;
            }

            List<PlanningEdge>.Enumerator e = _timeLevels[level - 1].mEdges.GetEnumerator();
            while (e.MoveNext())
            {
                if (e.Current.to == node)
                {
                    return true;
                }
            }
            return false;
        }

        public bool hasOut(PlanningNode node, int level)
        {
            if (level < 0 || level >= _planningLength - 1)
            {
                return false;
            }

            List<PlanningEdge>.Enumerator e = _timeLevels[level].mEdges.GetEnumerator();
            while (e.MoveNext())
            {
                if (e.Current.from == node)
                {
                    return true;
                }
            }
            return true;
        }

        public bool hasEdge(PlanningNode from, PlanningNode to)
        {
            if (to.parent.Level != from.parent.Level + 1)
            {
                return false;
            }

            List<PlanningEdge>.Enumerator e = from.parent.mEdges.GetEnumerator();
            while (e.MoveNext())
            {
                if (e.Current.from == from && e.Current.to == to)
                {
                    return true;
                }
            }

            return false;
        }

        public PathPlanningGraph(PathPlanningGraph prevGraph)
        {
            _planningLength = prevGraph._planningLength;
            _timeLevels = (LevelPartite[])prevGraph._timeLevels.Clone();
        }

        public PathPlanningGraph(int planningLength)
        {
            _planningLength = planningLength;
            _timeLevels = new LevelPartite[_planningLength];
            for (int t = 0; t < _planningLength; t++)
            {
                _timeLevels[t] = new LevelPartite(t);
            }
        }

        public void AddPlanningNode(PlanningNode node, int t)
        {
            _timeLevels[t].AddPlanningNode(node);
        }

        public void RemovePlanningNode(PlanningNode node, int t)
        {
            if (t > 0)
            {
                for (int i = _timeLevels[t - 1].mEdges.Count - 1; i >= 0; i--)
                {
                    if (_timeLevels[t - 1].mEdges[i].to == node)
                    {
                        _timeLevels[t - 1].mEdges.RemoveAt(i);
                    }
                }
            }

            for (int i = _timeLevels[t].mEdges.Count - 1; i >= 0; i--)
            {
                if (_timeLevels[t].mEdges[i].from == node)
                {
                    _timeLevels[t].mEdges.RemoveAt(i);
                }
            }

            for (int i = _timeLevels[t].mNodes.Count - 1; i >= 0; i--)
            {
                if (_timeLevels[t].mNodes[i] == node)
                {
                    _timeLevels[t].mNodes.RemoveAt(i);
                }
            }
        }

        public void Draw(string filename = "PlanningGraph")
        {
            BidirectionalGraph<string, QuickGraph.Edge<string>> mG
                = new BidirectionalGraph<string, QuickGraph.Edge<string>>();

            for (int t = 0; t < _planningLength; t++)
            {
                List<PlanningNode>.Enumerator eN = _timeLevels[t].mNodes.GetEnumerator();
                while (eN.MoveNext())
                {
                    mG.AddVertex(eN.Current.GetName());
                }
            }

            for (int t = 0; t < _planningLength - 1; t++)
            {
                List<PlanningEdge>.Enumerator eE = _timeLevels[t].mEdges.GetEnumerator();
                while (eE.MoveNext())
                {
                    QuickGraph.Edge<string> e = new Edge<string>(eE.Current.from.GetName(), eE.Current.to.GetName());
                    mG.AddEdge(e);
                }
            }

            var graphviz = new GraphvizAlgorithm<string, Edge<string>>(mG);
            string dotFile = graphviz.Generate(new FileDotEngine(), filename);
            var diagramFile = dotFile.Replace(".dot", ".png");

            string graphOutput = string.Format(@"""{0}"" -o ""{1}"" -Tpng", dotFile, diagramFile);
            Process.Start(new ProcessStartInfo("dot", graphOutput) { CreateNoWindow = true, UseShellExecute = false });
        }

        public void PrintPlanningGraph(double[,] entropy)
        {
            for (int l = 0; l < _planningLength; l++)
            {
                string levelInfo = "Level (" + l.ToString() + "): ";
                List<PlanningNode>.Enumerator e = this[l].mNodes.GetEnumerator();
                while (e.MoveNext())
                {
                    int posX = e.Current.pos.X;
                    int posY = e.Current.pos.Y;
                    levelInfo += "[" + posX.ToString() + ","
                        + posY.ToString() + "] = " + entropy[posX, posY].ToString() + " ";
                }
                Console.WriteLine(levelInfo);
            }
        }

        public long GetExpandingNodeNumber()
        {
            long count = 1;
            Matrix cumMatrix = new Matrix(1, 1);
            cumMatrix[0, 0] = 1;

            for (int l = 0; l < _planningLength - 1; l++)
            {
                int rowNum = _timeLevels[l].mNodes.Count;
                int colNum = _timeLevels[l + 1].mNodes.Count;

                Matrix connectMatrix = new Matrix(rowNum, colNum);

                for (int i = 0; i < rowNum; i++)
                {                    
                    List<PlanningEdge> edges = _timeLevels[l].GetEdges(_timeLevels[l].mNodes[i]);

                    List<PlanningEdge>.Enumerator enumEdge = edges.GetEnumerator();
                    while (enumEdge.MoveNext())
                    {
                        int j = _timeLevels[l + 1].GetIndex(enumEdge.Current.to);
                        connectMatrix[i, j] = 1;
                    }
                }

                cumMatrix = cumMatrix.Multiply(connectMatrix);

                for (int r = 0; r < cumMatrix.RowCount; r++)
                {
                    for (int c = 0; c < cumMatrix.ColumnCount; c++)
                    {
                        count += (int)cumMatrix[r, c];
                    }
                }

            }

            return count;
        }
    }
}