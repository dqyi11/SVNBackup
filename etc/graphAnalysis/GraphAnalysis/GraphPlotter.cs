using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using QuickGraph;
//using QuickGraph.Graphviz;
//using System.Diagnostics;
using Microsoft.Glee;
using QuickGraph.Glee;
using Microsoft.Glee.Drawing;
using System.Windows.Forms;



namespace GraphAnalysis
{
    class GraphPlotter
    {
        public Graph mGraph;

        public GraphPlotter(Graph graph)
        {
            mGraph = graph;
        }

        public void Draw()
        {

            UndirectedGraph<Vertex, QuickGraph.Edge<Vertex>> mG
                = new UndirectedGraph<Vertex, QuickGraph.Edge<Vertex>>();

            List<Vertex>.Enumerator etV = mGraph.mVertices.GetEnumerator();
            while (etV.MoveNext())
            {
                mG.AddVertex(etV.Current);
            }

            List<Edge>.Enumerator etE = mGraph.mEdges.GetEnumerator();
            while (etE.MoveNext())
            {
                QuickGraph.Edge<Vertex> e = new Edge<Vertex>(etE.Current.mVertexA, etE.Current.mVertexB);
                mG.AddEdge(e);
            }
            
            Console.WriteLine("QUICK GRAPH FORMAT:");
            Console.WriteLine(mG.VertexCount);
            Console.WriteLine(mG.EdgeCount);

            /*
            var graphviz = new GraphvizAlgorithm<Vertex, Edge<Vertex>>(mG);
            string output = graphviz.Generate(new FileDotEngine(), "graph");

            Console.WriteLine("OUTPUT");
            Console.WriteLine(output);

            // Process drawProcess = new Process();
            Process.Start("dot","-Tpng graph.dot > graph.png");
            */

            var populator = GleeGraphUtility.Create<Vertex, Edge<Vertex>>(mG);
            populator.Compute();
            Microsoft.Glee.Drawing.Graph gleeG = populator.GleeGraph;

            //create a viewer object
            Microsoft.Glee.GraphViewerGdi.GViewer viewer = new Microsoft.Glee.GraphViewerGdi.GViewer();
            //create a form
            System.Windows.Forms.Form form = new System.Windows.Forms.Form();

            //bind the graph to the viewer
            viewer.Graph = gleeG;

            //associate the viewer with the form
            form.SuspendLayout();
            viewer.Dock = System.Windows.Forms.DockStyle.Fill;
            form.Controls.Add(viewer);
            form.ResumeLayout();

            //show the form
            form.ShowDialog();

            
        }
    }
}
