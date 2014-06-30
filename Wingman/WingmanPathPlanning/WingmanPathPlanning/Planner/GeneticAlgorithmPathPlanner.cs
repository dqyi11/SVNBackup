using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Hexagonal;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Base;

namespace WingmanPathPlanning.Planner
{
    class GeneticAlgorithmPathPlanner : PathPlanner 
    {
        double punishFactor;
        int chromeNum;
        int bestChromeNum;
        int crossoverNum;
        int runTimes;
        int[][] chromes;
        int[][] newChromes;
        double[] scores;

        public GeneticAlgorithmPathPlanner(HexagonalMap map, Robot agent)
            : base(map, agent)
        {
            punishFactor = 20.0;
            chromeNum = 200;
            bestChromeNum = 60;
            crossoverNum = 40;
            runTimes = 500;

            scores = new double[chromeNum];
            chromes = new int[chromeNum][];
            newChromes = new int[chromeNum][];
        }

        double scoreFunction(HexaPath path, PathPlanningGraph graph)
        {
            double score = _agent.Score(path, _localEntropy);

            int pathLength = path.Length;
            for (int i = 0; i < pathLength-1; i++)
            {
                if (false == _map.IsAccessible(path[i], path[i + 1]))
                {
                    score -= punishFactor;
                }
            }
            return score;
        }

        HexaPath GetPath(int[] indexSeq, PathPlanningGraph graph)
        {
            HexaPath path = new HexaPath();
            int pathLength = graph.planningLength;
            for(int l=0;l<pathLength;l++)
            {
                path.AddPos(graph[l].mNodes[indexSeq[l]].pos);
            }
            return path;
        }

        double ScorePath(int[] indexSeq, PathPlanningGraph graph)
        {
            HexaPath path = GetPath(indexSeq, graph);
            return scoreFunction(path, graph);
        }

        int[] GetBestIndex(double[] scores)
        {
            int [] bestIndex = new int[bestChromeNum];
            int[] fullIndex = new int[chromeNum];
            double[] localScores = (double[])scores.Clone();
            for (int i = 0; i < chromeNum; i++)
            {
                fullIndex[i] = i;
            }

            int j = chromeNum;
            double temp = 0.0;
            int tempIndex = 0;
            while (j > 0)
            {
                for (int k = 0; k < j - 1; k++)
                {
                    if (localScores[k] > localScores[k + 1])
                    {
                        temp = localScores[k];
                        localScores[k] = localScores[k + 1];
                        localScores[k + 1] = temp;

                        tempIndex = fullIndex[k];
                        fullIndex[k] = fullIndex[k + 1];
                        fullIndex[k + 1] = tempIndex;
                    }
                }
                j--;
            }

            for (int i = 0; i < bestChromeNum; i++)
            {
                bestIndex[i] = fullIndex[i];
            }

            return bestIndex;
        }

        public override HexaPath FindPath(PathPlanningGraph graph, HexaPos start)
        {
            HexaPath path = new HexaPath();
            Random rnd = new Random();

            for(int i=0;i<chromeNum;i++)
            {
                chromes[i] = new int[graph.planningLength];
            }

            for (int i = 0; i < chromeNum; i++)
            {
                for (int j = 0; j < graph.planningLength; j++)
                {
                    chromes[i][j] = rnd.Next(graph[j].mNodes.Count);
                }
            }

            for (int r = 0; r < runTimes; r++)
            {
                for (int i = 0; i < chromeNum; i++)
                {
                    scores[i] = ScorePath(chromes[i], graph);
                }

                int [] bestIndex = GetBestIndex(scores);

                for (int i = 0; i < bestChromeNum; i++)
                {
                    newChromes[i] = (int[])chromes[bestIndex[i]].Clone();
                }

                // crossover
                for (int i = bestChromeNum; i < bestChromeNum + crossoverNum; i+=2)
                {
                    int crossPos = rnd.Next(1, graph.planningLength - 1);
                    int crossIdxA = rnd.Next(bestChromeNum - 1);
                    int crossIdxB = rnd.Next(bestChromeNum - 1);
                    newChromes[i] = new int[graph.planningLength];
                    newChromes[i+1] = new int[graph.planningLength];

                    for (int j = 0; j < crossPos; j++)
                    {
                        newChromes[i][j] = chromes[crossIdxA][j];
                        newChromes[i + 1][j] = chromes[crossIdxB][j];
                    }

                    for (int j = crossPos; j < graph.planningLength; j++)
                    {
                        newChromes[i][j] = chromes[crossIdxB][j];
                        newChromes[i + 1][j] = chromes[crossIdxA][j];
                    }
                }

                // mutate
                for (int i = bestChromeNum + crossoverNum; i < chromeNum; i ++)
                {
                    int mutIdx = rnd.Next(bestChromeNum - 1);
                    newChromes[i] = (int[])chromes[mutIdx].Clone();
                    int mutPos = rnd.Next(1, graph.planningLength - 1);
                    int mutVal = rnd.Next(0, graph[mutPos].mNodes.Count - 1);
                    newChromes[i][mutPos] = mutVal;
                }

                chromes = (int[][])newChromes.Clone();
            }

            path = GetPath(chromes[0], graph);
            return path;
        }
    }
}
