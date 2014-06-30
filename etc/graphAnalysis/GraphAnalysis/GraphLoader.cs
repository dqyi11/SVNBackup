using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Xml;
using QuickGraph;

namespace GraphAnalysis
{
    class GraphLoader
    {
        protected string mFilename;
        protected XmlDocument xDoc;

        protected Graph mGraph;

        AdjacencyGraph<string, Edge<string>> qGraph;


        public GraphLoader(string filename)
        {
            mFilename = filename;
            xDoc = new XmlDocument();
            xDoc.Load(mFilename);
        }

        public Graph GetGraph()
        {
            return mGraph;
        }

        public void Init()
        {
            XmlNode graphNode =
                xDoc.SelectSingleNode("Graph");
          
            mGraph = new Graph();
            qGraph = new AdjacencyGraph<string, Edge<string>>(true);

            XmlNodeList partiteList =
                graphNode.SelectNodes("Partite");

            foreach (XmlNode nodeP in partiteList)
            {
                Partite partite = mGraph.CreatePartite(nodeP.Attributes
                    .GetNamedItem("name").Value);

                XmlNodeList vertexList =
                    nodeP.SelectNodes("Vertex");

                foreach (XmlNode nodeV in vertexList)
                {
                    Vertex vertex = mGraph.CreateVertex(nodeV.Attributes
                        .GetNamedItem("name").Value);

                    partite.AddVertex(vertex);

                    qGraph.AddVertex(nodeV.Attributes
                        .GetNamedItem("name").Value);
                }
                
            }

            XmlNodeList connectorList = graphNode.SelectNodes("Connector");

            foreach (XmlNode nodeC in connectorList)
            {
                Partite partiteA = mGraph.GetPartite(nodeC.Attributes
                    .GetNamedItem("partite1").Value);
                Partite partiteB = mGraph.GetPartite(nodeC.Attributes
                    .GetNamedItem("partite2").Value);
                Connector connector = mGraph.Connect(partiteA, partiteB);

                XmlNodeList edgeList =
                    nodeC.SelectNodes("Edge");

                foreach (XmlNode nodeE in edgeList)
                {
                    Vertex vertexA = mGraph.GetVertex(nodeE.Attributes
                        .GetNamedItem("node1").Value);
                    Vertex vertexB = mGraph.GetVertex(nodeE.Attributes
                        .GetNamedItem("node2").Value);
                    Edge edge = mGraph.Connect(vertexA, vertexB);

                    connector.AddEdge(edge);

                    Edge<string> a_e = new Edge<string>(nodeE.Attributes
                        .GetNamedItem("node1").Value,
                        nodeE.Attributes
                        .GetNamedItem("node2").Value);
                    qGraph.AddEdge(a_e);
                }

            }

            foreach (var v in qGraph.Vertices)
                Console.WriteLine("VERTEX: " + v);
            foreach (var e in qGraph.Edges)
                Console.WriteLine("EDGE: " + e);
        }


    }
}
