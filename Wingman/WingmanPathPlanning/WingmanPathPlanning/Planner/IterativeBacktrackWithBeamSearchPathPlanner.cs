using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Hexagonal;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Base;

namespace WingmanPathPlanning.Planner
{
    class IterativeBacktrackWithBeamSearchPathPlanner: PathPlanner
    {
        int beamNum = 10;
        HexaPath[] beamPaths;
        double[] beamScores;

        public IterativeBacktrackWithBeamSearchPathPlanner(HexagonalMap map, Robot agent)
            : base(map, agent)
        {
            beamPaths = new HexaPath[beamNum*2];
            beamScores = new double[beamNum*2];
        }

        public override HexaPath FindPath(PathPlanningGraph graph, HexaPos start)
        {
            //initial first beamNum of the paths
            for (int i = 0; i < beamNum; i++)
            {
                beamPaths[i].AddPos(start);
            }
            
            GenerateAllBeamPaths(graph);

            
            return beamPaths[0];
        }

        void GenerateAllBeamPaths(PathPlanningGraph graph)
        {
            // stop till all subpath has been length of T
            bool stopCriteria = false;
            while (!stopCriteria)
            {
                

                stopCriteria = CheckStopCriteria(graph);
            }
        }

        bool CheckStopCriteria(PathPlanningGraph graph)
        {
            for (int i = 0; i < beamNum; i++)
            {
                if (beamPaths[i].Length < graph.planningLength)
                {
                    return false;
                }
            }
            return true;
        }

        double ScoreSubpath(HexaPath subpath, PathPlanningGraph graph)
        {
            double score = 0.0;
            // score a subpath



            return score;
        }

        void SortBeamPath(PathPlanningGraph graph)
        {
            for (int i = 0; i < beamNum; i++)
            {
                for (int j = 0; j < beamNum - i; j++)
                {
                    if (beamScores[j - 1] < beamScores[j])
                    {
                        SwapScore(beamScores[j - 1], beamScores[j]);
                        SwapPath(beamPaths[j - 1], beamPaths[j]);
                    }
                }
            }
        }

        void SwapScore(double a, double b)
        {
            double temp = a;
            a = b;
            b = temp;
        }

        void SwapPath(HexaPath pathA, HexaPath pathB)
        {
            HexaPath tempPath = pathA;
            pathA = pathB;
            pathB = tempPath;
        }
    }
}
