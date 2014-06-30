using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using DotNumerics.LinearAlgebra;

namespace GraphAnalysis
{
    class Program
    {
        static void Main(string[] args)
        {
            GraphLoader loader = new GraphLoader("completPartite.xml");
            loader.Init();
            Graph graph = loader.GetGraph();

            //graph.PrintVertex();
            //graph.PrintEdge();

            Matrix A = graph.GetAdjacentMatrix();
            Matrix D = graph.GetDegreeMatrix();

            Console.WriteLine("ADJACENT MATRIX:");
            Console.WriteLine(A.MatrixToString());

            Console.WriteLine("DEGREE MATRIX:");
            Console.WriteLine(D.MatrixToString());

            Matrix L = D.Subtract(A);

            Console.WriteLine("LAPLACIAN MATRIX:");
            Console.WriteLine(L.MatrixToString());

            EigenSystem eigSys = new EigenSystem();

            ComplexMatrix eigenVals = eigSys.GetEigenvalues(L);

            Console.WriteLine("EIGENVALUES:");
            Console.WriteLine(eigenVals.MatrixToString());

            GraphPlotter plotter = new GraphPlotter(graph);
            plotter.Draw();

            return;
        }
    }
}
