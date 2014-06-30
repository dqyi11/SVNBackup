using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using WingmanPathPlanning.Hexagonal;
using WingmanPathPlanning.Base;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Planner;

namespace WingmanPathPlanning
{
    public class TeleportParameter
    {
        public int runIndex;

        public double maxScore;
        public long problemSize;
        public long exploredSize;
        public double scoreAtFirstRun;
        public long totalRunTime;
        public long hitOptimalRunTime;
        public double teleportScore;
        public double greedyScore;

        public int wingmanRange;
        public int humanRange;
        public int robotRange;

    }

    public class TeleportTester
    {
        MapForm _mapForm;

        /*
        int [] wingmanRange = { 2, 3, 4, 5, 6, 7 };
        int [] robotRange = { 0, 1, 2, 3, 4, 5 };
        int [] humanRange = { 0, 1, 2, 3, 4, 5 };
         */

        int[] wingmanRange = { 2 };
        int[] robotRange = { 2 };
        int[] humanRange = { 2 };

        int trial_times = 20;

        string prefix = "PM";

        public List<TeleportParameter> performanceList;

        public TeleportTester(MapForm form)
        {
            _mapForm = form;
            performanceList = new List<TeleportParameter>();
        }

        public void save(string prefix)
        {
            string filename2 = prefix + "-ScoreAtFirstRun-" + ".csv";
            string filename3 = prefix + "-GdScoreAtFirstRun-" + ".csv";
            string filename4 = prefix + "-TpScoreAtFirstRun-" + ".csv";
            string filename5 = prefix + "-Info-" + ".txt";

            using (StreamWriter sw = new StreamWriter(filename2))
            {
                Console.WriteLine("");
                Console.WriteLine("Output... " + filename2);
                foreach (TeleportParameter per in performanceList)
                {
                    Console.WriteLine(per.scoreAtFirstRun.ToString() + ", ");
                    sw.Write(per.scoreAtFirstRun.ToString() + ", ");
                }
                Console.WriteLine("");
            }

            using (StreamWriter sw = new StreamWriter(filename3))
            {
                Console.WriteLine("");
                Console.WriteLine("Output... " + filename3);
                foreach (TeleportParameter per in performanceList)
                {
                    Console.WriteLine(per.greedyScore.ToString() + ", ");
                    sw.Write(per.greedyScore.ToString() + ", ");
                }
                Console.WriteLine("");
            }

            using (StreamWriter sw = new StreamWriter(filename4))
            {
                Console.WriteLine("");
                Console.WriteLine("Output... " + filename4);
                foreach (TeleportParameter per in performanceList)
                {
                    Console.WriteLine(per.teleportScore.ToString() + ", ");
                    sw.Write(per.teleportScore.ToString() + ", ");
                }
                Console.WriteLine("");
            }

            using (StreamWriter sw = new StreamWriter(filename5))
            {
                Console.WriteLine("");
                Console.WriteLine("Output... " + filename5);
                foreach (TeleportParameter per in performanceList)
                {
                    Console.WriteLine("Idx: " + per.runIndex.ToString() + " WR: " + per.wingmanRange.ToString()
                        + " RR: " + per.robotRange.ToString() + " HR: " + per.humanRange.ToString());
                    sw.WriteLine("Idx: " + per.runIndex.ToString() + " WR: " + per.wingmanRange.ToString()
                        + " RR: " + per.robotRange.ToString() + " HR: " + per.humanRange.ToString());
                }
                Console.WriteLine("");
            }
        }

        public void Run()
        {
            for (int i = 0; i < trial_times; i++)
            {
                foreach (int WR in wingmanRange)
                {
                    HexaPos startPos = this._mapForm.human.path[0];

                    TopologyGraphGenerator topologyGenerator = new TopologyGraphGenerator(this._mapForm.map);

                    TopologyGraph tograph = topologyGenerator.GetTopologyGraph();

                    PlanningGraphGenerator planningGenerator = new PlanningGraphGenerator(tograph);
                    PathPlanningGraph planGraph = planningGenerator.GetPathPlanningGraph(this._mapForm.HumanPath, WR);

                    PlanningGraphPruner pruner = new PlanningGraphPruner();
                    PathPlanningGraph newPlanGraph = pruner.GetPlanningGraph(planGraph, startPos);

                    PathPlanningGraph prunedPlanGraph = pruner.BackwardPrune(newPlanGraph);
                    prunedPlanGraph = pruner.ForwardPrune(prunedPlanGraph);

                    Robot robot = new Robot(this._mapForm.map);
                    Human human = new Human(this._mapForm.map);

                    foreach (int HR in humanRange)
                    {

                        this._mapForm.map.GetMapStateMgr().RandomizeValue();

                        human.SetObservationRange(HR);
                        double[,] localEntropy = this._mapForm.map.GetMapStateMgr().CopyEntropy();
                        human.Update(this._mapForm.HumanPath, localEntropy);

                        foreach (int RR in robotRange)
                        {
                            robot.SetObservationRange(RR);
                            Console.WriteLine("Trial Time: " + i.ToString() + " WR: " + WR.ToString() + " RR: " + RR.ToString() + " HR: " + HR.ToString());

                            HexaPath maxPath = new HexaPath();

                            TreeExpandingWithIterativeTrackingPathPlanner planner
                        = new TreeExpandingWithIterativeTrackingPathPlanner(this._mapForm.map, robot);
                            planner._localEntropy = localEntropy;
                            planner.iteratingOnce = true;

                            maxPath = planner.FindPath(prunedPlanGraph, startPos);

                            Console.WriteLine("PATH : " + maxPath.ToString());
                            // double[,] tempEntropy = (double[,])this._mapForm.map.GetMapStateMgr().CopyEntropy();
                            //double maxScore = robot.Score(maxPath, tempEntropy);
                            double maxScore = planner.finalMaxScore;
                            Console.WriteLine("SCORE: " + maxScore);
                            
                            Console.WriteLine("");

                            HexaPath tpPath = new HexaPath();

                            TeleportPathPlanner planner2 = new TeleportPathPlanner(this._mapForm.map, robot);
                            planner2._localEntropy = (double[,])localEntropy.Clone();
                            tpPath = planner2.FindPath(prunedPlanGraph, startPos);

                            HexaPath gdPath = new HexaPath();

                            SimpleGreedyPathPlanner planner3 = new SimpleGreedyPathPlanner(this._mapForm.map, robot);
                            planner3._localEntropy = (double[,])localEntropy.Clone();
                            gdPath = planner3.FindPath(prunedPlanGraph, startPos);
                    

                            TeleportParameter param = new TeleportParameter();
                            param.maxScore = maxScore;
                            param.problemSize = planner.problemSize;
                            param.scoreAtFirstRun = planner.scoreAtFirstRun;
                            param.exploredSize = planner.exploredSize;
                            param.totalRunTime = planner.totalRunTime;
                            param.hitOptimalRunTime = planner.hitOptimalRunTime;
                            
                            param.teleportScore = planner2.ScorePath(robot, (double[,])localEntropy.Clone(), tpPath);
                            param.greedyScore = planner3.ScorePath(robot, (double[,])localEntropy.Clone(), gdPath);
                   
                            param.robotRange = RR;
                            param.humanRange = HR;
                            param.wingmanRange = WR;
                            param.runIndex = i;

                            this.performanceList.Add(param);

                        }
                    }
                }
            }

            save(prefix);
        }
    }
}
