using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using PathPlanner.Hexagonal;
using QuickGraph;
using QuickGraph.Graphviz;
using QuickGraph.Graphviz.Dot;
using QuickGraph.Collections;
using System.Diagnostics;
using System.Drawing;

namespace PathPlanner.Planner
{
    public class ExpandingNode
    {
        public PlanningNode planningNode;
        public ExpandingNode parentNode;
        public List<ExpandingNode> childrenNodes;

        public double minVal;
        public double maxVal;
        public int level;

        public double instRwd;
        public double futrRwd;

        public enum STATE
        {
            NEW = 1,
            EXPANDED,
            FREEZED,
        };

        public STATE state;

        public ExpandingNode(PlanningNode node)
        {
            planningNode = node;
            parentNode = null;
            childrenNodes = new List<ExpandingNode>();
            minVal = 0.0;
            maxVal = Double.MaxValue;
            level = -1;
            instRwd = 0.0;
            futrRwd = 0.0;
            state = STATE.NEW;
        }

        public string GetName()
        {
            string name = planningNode.GetName();
            return name;
        }

        public string GetNameWithParent()
        {
            string name = planningNode.GetName();

            if (parentNode != null)
            {
                name += "++";
                name += parentNode.planningNode.GetName();
            }

            return name;
        }

        public void AddToParent(ExpandingNode parent)
        {
            parentNode = parent;
            if (parent != null)
            {
                parent.childrenNodes.Add(this);
                level = parent.level + 1;
            }
            else
            {
                level = 0;
            }
        }
    }

    public class ExpandingTree
    {
        public ExpandingNode rootNode;
        public List<ExpandingNode> nodeList;

        public ExpandingTree(ExpandingNode root)
        {
            nodeList = new List<ExpandingNode>();
            rootNode = root;
            root.AddToParent(null);
            nodeList.Add(root);
        }

        public ExpandingNode GetRoot()
        {
            return rootNode;
        }

        public int nodeNum
        {
            get
            {
                return nodeList.Count;
            }
        }

        public void AddToParent(ExpandingNode node, ExpandingNode parent)
        {
            if (parent != null)
            {
                node.AddToParent(parent);
                nodeList.Add(node);
            }
        }

        public int GetNewNodeCountByLevel(int levelNum)
        {
            int nodeCnt = 0;
            List<ExpandingNode>.Enumerator e = nodeList.GetEnumerator();
            while (e.MoveNext())
            {
                if (e.Current.level == levelNum && e.Current.state == ExpandingNode.STATE.NEW)
                {
                    nodeCnt++;
                }
            }
            return nodeCnt;
        }

        public ExpandingNode GetMaxLeafNode(int stopLevel)
        {
            List<ExpandingNode>.Enumerator e = nodeList.GetEnumerator();
            double maxVal = -0.1;
            ExpandingNode maxLeafNode = null;
            while (e.MoveNext())
            {
                // if it is less than stop level and child count = 0
                if (e.Current.level < stopLevel && e.Current.childrenNodes.Count == 0)
                {
                    // only expanding NEW node
                    if (e.Current.state == ExpandingNode.STATE.NEW)
                    {
                        if (e.Current.maxVal > maxVal)
                        {
                            maxLeafNode = e.Current;
                        }
                    }
                }
            }
            return maxLeafNode;
        }

        public void Freeze(double freezeThreshold)
        {
            // go through the entire list to find which nodes should be freezed
            List<ExpandingNode>.Enumerator eV = nodeList.GetEnumerator();
            while (eV.MoveNext())
            {
                if (eV.Current.state == ExpandingNode.STATE.NEW)
                {
                    if (eV.Current.maxVal <= freezeThreshold)
                    {
                        eV.Current.state = ExpandingNode.STATE.FREEZED;
                        eV.Current.minVal = freezeThreshold;
                    }
                }
            }
        }

        public ExpandingNode GetMaxChild(ExpandingNode node)
        {
            ExpandingNode maxChild = null;
            double maxVal = -0.1;

            List<ExpandingNode>.Enumerator e = node.childrenNodes.GetEnumerator();
            while (e.MoveNext())
            {
                if (e.Current.maxVal > maxVal)
                {
                    maxChild = e.Current;
                    maxVal = e.Current.maxVal;
                }
            }

            return maxChild;
        }

        public void BackPropagateMinVal(ExpandingNode node, double val)
        {
            // set self minVal
            node.minVal = val;

            ExpandingNode parent = node.parentNode;

            // stop at the root
            while (parent != null && parent.minVal < val)
            {
                parent.minVal = val;
                parent = parent.parentNode;
            }
        }

        public HexaPath GetPath(ExpandingNode node)
        {
            HexaPath path = new HexaPath();
            
            ExpandingNode currentNode = node;
            while (currentNode != null)
            {
                path.InsertFront(currentNode.planningNode.pos);
                currentNode = currentNode.parentNode;
            }
            
            return path;
        }

        public void Draw(string filename = "ExpandingTree")
        {
            BidirectionalGraph<ExpandingNode, QuickGraph.Edge<ExpandingNode>> mG
                = new BidirectionalGraph<ExpandingNode, QuickGraph.Edge<ExpandingNode>>();

            List<ExpandingNode>.Enumerator eV = nodeList.GetEnumerator();
            while (eV.MoveNext())
            {
                mG.AddVertex(eV.Current);
            }


            List<ExpandingNode>.Enumerator eV2 = nodeList.GetEnumerator();
            while (eV2.MoveNext())
            {
                List<ExpandingNode>.Enumerator eV3 = eV2.Current.childrenNodes.GetEnumerator();
                while (eV3.MoveNext())
                {
                    QuickGraph.Edge<ExpandingNode> e = new Edge<ExpandingNode>(eV2.Current, eV3.Current);
                    mG.AddEdge(e);
                }
            }

            var graphviz = new GraphvizAlgorithm<ExpandingNode, Edge<ExpandingNode>>(mG);
            //graphviz.CommonVertexFormat.Shape = GraphvizVertexShape.Record;
            graphviz.FormatVertex += new FormatVertexEventHandler<ExpandingNode>(FormatVertex);


            string dotFile = graphviz.Generate(new FileDotEngine(), filename);
            var diagramFile = dotFile.Replace(".dot", ".png");

            string graphOutput = string.Format(@"""{0}"" -o ""{1}"" -Tpng", dotFile, diagramFile);
            Process.Start(new ProcessStartInfo("dot", graphOutput) { CreateNoWindow = true, UseShellExecute = false });

        }

        private void FormatVertex(object sender, FormatVertexEventArgs<ExpandingNode> e)
        {
            ExpandingNode ent = e.Vertex;
            //e.VertexFormatter.Label = ent.GetName();
            e.VertexFormatter.Shape = GraphvizVertexShape.Record;
            e.VertexFormatter.Style = GraphvizVertexStyle.Filled;

            switch (ent.state)
            {
                case ExpandingNode.STATE.NEW:
                    e.VertexFormatter.FillColor = Color.Orange;
                    break;
                case ExpandingNode.STATE.EXPANDED:
                    e.VertexFormatter.FillColor = Color.Red;
                    break;
                case ExpandingNode.STATE.FREEZED:
                    e.VertexFormatter.FillColor = Color.LightBlue;
                    break;
            }

            GraphvizRecord rec = new GraphvizRecord();

            GraphvizRecordCell name = new GraphvizRecordCell();
            name.Text = ent.GetName();

            GraphvizRecordCell maxScore = new GraphvizRecordCell();
            maxScore.Text = ent.maxVal.ToString();

            GraphvizRecordCell minScore = new GraphvizRecordCell();
            minScore.Text = ent.minVal.ToString();

            rec.Cells.Add(name);
            rec.Cells.Add(maxScore);
            rec.Cells.Add(minScore);

            e.VertexFormatter.Record = rec;
        }
     
    }
}
