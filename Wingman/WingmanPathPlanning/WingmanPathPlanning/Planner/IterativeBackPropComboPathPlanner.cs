using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Hexagonal;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Base;

namespace WingmanPathPlanning.Planner
{
    class Rewards
    {
        public double[][] totalRewards;
        public double[][] instantRewards;
        public double[][] futureRewards;
        public int len;

        public Rewards(PathPlanningGraph graph)
        {
            len = graph.planningLength;
            totalRewards = new double[len][];
            instantRewards = new double[len][];
            futureRewards = new double[len][];
            for (int t = 0; t < graph.planningLength; t++)
            {
                int nodeNum = graph[t].mNodes.Count;
                totalRewards[t] = new double[nodeNum];
                instantRewards[t] = new double[nodeNum];
                futureRewards[t] = new double[nodeNum];
            }
        }

        public Rewards(Rewards other)
        {
            len = other.len;
            totalRewards = (double[][])(other.totalRewards.Clone());
            instantRewards = (double[][])(other.instantRewards.Clone());
            futureRewards = (double[][])(other.futureRewards.Clone());
        }
    }

    class IterativeBackPropComboPathPlanner : PathPlanner 
    {
        Rewards _estimated;
        Rewards _optEstimated;
        Rewards _pesEstimated;

        public IterativeBackPropComboPathPlanner(HexagonalMap map, Robot agent)
            : base(map, agent)
        {
        }

        public override HexaPath FindPath(PathPlanningGraph graph, HexaPos start)
        {
            int planningLength = graph.planningLength;
            _estimated = new Rewards(graph);

            HexaPath path = new HexaPath();
            
            _optEstimated = new Rewards(_estimated);
            _pesEstimated = new Rewards(_estimated);
            double[,] optEntropy = (double[,])(_localEntropy.Clone());
            double[,] pesEntropy = (double[,])(_localEntropy.Clone());

            HexaPath optMaxPath = new HexaPath();
            HexaPath pesMaxPath = new HexaPath();
            optMaxPath.AddPos(start);
            pesMaxPath.AddPos(start);

            for (int t = 1; t < planningLength; t++)
            {
                // get path for opt
                _agent.Update(optMaxPath, optEntropy);
                UpdateRewardOptEst(_optEstimated, optEntropy, graph, optMaxPath);
                HexaPos nextOptPos = GetMax(_optEstimated, graph, optMaxPath);
                optMaxPath.AddPos(nextOptPos);

                // get path for pes
                _agent.Update(pesMaxPath, pesEntropy);
                UpdateRewardPesEst(_pesEstimated, pesEntropy, graph, pesMaxPath);
                HexaPos nextPesPos = GetMax(_pesEstimated, graph, pesMaxPath);
                pesMaxPath.AddPos(nextPesPos);
            }

            double optMaxScore = _agent.Score(optMaxPath, _localEntropy);
            double pesMaxScore = _agent.Score(pesMaxPath, _localEntropy);

            int maxTryCnt = 10;
            bool converged = false;
            int tryCnt = 0;

            while (converged == false && tryCnt <= maxTryCnt)
            {
                tryCnt++;

                if (pesMaxScore >= optMaxScore)
                {
                    path = pesMaxPath;
                    converged = true;
                }
                else
                {
                    path = optMaxPath;

                    // correct the estimation at step t
                    int diffFrom = pesMaxPath.DifferentAt(optMaxPath);
                    HexaPath subpath = optMaxPath.SubPath(diffFrom, optMaxPath.Length - 1);
                    PlanningNode diffNode = graph[diffFrom].GetNode(subpath[0]);
                    int diffIdx = graph[diffFrom].GetIndex(diffNode);
                    HexaPath prevPath = optMaxPath.SubPath(0, diffFrom-1);
                    double[,] tempEntropy = (double[,])(_localEntropy.Clone());
                    _agent.Update(prevPath, tempEntropy);
                    _pesEstimated.totalRewards[diffFrom][diffIdx] = _agent.Score(subpath, tempEntropy);

                    HexaPath newCandidatePath = new HexaPath();
                    newCandidatePath.AddPos(start);

                    HexaPath newSubCandidate = EstimatePath(graph, 1, start, _pesEstimated);
                    newCandidatePath.Merge(newSubCandidate);

                    double newCandidateScore = _agent.Score(newSubCandidate, _localEntropy);

                    if (newCandidateScore <= pesMaxScore)
                    {
                        converged = true;
                    }
                    else
                    {
                        pesMaxScore = newCandidateScore;
                        pesMaxPath = newCandidatePath;
                    }

                }
            }

            return path;
        }

        void UpdateRewardOptEst(Rewards reward, double[,] entropy, PathPlanningGraph graph, HexaPath path)
        {
            int currentLen = path.Length;
            int totalLen = graph.planningLength;
            int num = graph[currentLen].mNodes.Count;
            for (int i = 0; i < num; i++)
            {
                PlanningNode node = graph[currentLen].mNodes[i];
                HexaPath newpath = new HexaPath();
                newpath.AddPos(node.pos);
                double[,] localEntropy = (double[,])entropy.Clone();
                _agent.Update(newpath, localEntropy);
                Rewards newreward = new Rewards(reward);
                // update estimation
                UpdateEstimation(newreward, localEntropy, graph, currentLen, totalLen - 1);
                // backtrack
                Backtrack(newreward, graph, totalLen - 2, currentLen);
                reward.instantRewards[currentLen][i] = newreward.instantRewards[currentLen][i];
                reward.futureRewards[currentLen][i] = newreward.futureRewards[currentLen][i];
                reward.totalRewards[currentLen][i] = newreward.totalRewards[currentLen][i];
            }
        }

        void UpdateRewardPesEst(Rewards reward, double[,] entropy, PathPlanningGraph graph, HexaPath path)
        {
            int currentLen = path.Length;
            int totalLen = graph.planningLength;
            int num = graph[currentLen].mNodes.Count;
            for (int i = 0; i < num; i++)
            {
                PlanningNode node = graph[currentLen].mNodes[i];
                HexaPath newpath = new HexaPath();
                newpath.AddPos(node.pos);
                double[,] localEntropy = (double[,])entropy.Clone();
                // instant reward
                //reward.instantRewards[currentLen][i] = _agent.Score(newpath, entropy, graph);
                reward.instantRewards[currentLen][i] = GetEstimation(_agent, localEntropy, node.pos, _map);

                // future reward
                _agent.Update(newpath, localEntropy);
                Rewards newreward = new Rewards(reward);
                // update estimation
                UpdateEstimation(newreward, localEntropy, graph, currentLen+1, totalLen - 1);
                // backtrack
                Backtrack(newreward, graph, totalLen - 1, currentLen+1);

                HexaPath estPath = EstimatePath(graph, currentLen+1, node.pos, newreward);
                double futureScore = 0.0;
                if (estPath.Length > 0)
                {
                    futureScore = _agent.Score(estPath, localEntropy);
                }
                reward.futureRewards[currentLen][i] = futureScore;
                reward.totalRewards[currentLen][i] = reward.instantRewards[currentLen][i]
                    + reward.futureRewards[currentLen][i];
            }
        }

        HexaPath EstimatePath(PathPlanningGraph graph, int currentLevel, HexaPos lastPos, Rewards reward)
        {
            HexaPath newpath = new HexaPath();
            int endLevel = graph.planningLength - 1;

            for (int t = currentLevel; t <= endLevel; t++)
            {
                PlanningNode lastNode = graph[t-1].GetNode(lastPos);
                List<PlanningEdge> edges = graph[t-1].GetEdges(lastNode);
                double maxVal = -0.01;
                HexaPos maxPos = null;
                List<PlanningEdge>.Enumerator e = edges.GetEnumerator();
                while (e.MoveNext())
                {
                    int nextIdx = graph[t].GetIndex(e.Current.to);
                    if (reward.totalRewards[t][nextIdx] > maxVal)
                    {
                        maxPos = graph[t].mNodes[nextIdx].pos;
                        maxVal = reward.totalRewards[t][nextIdx];
                    }
                }
                newpath.AddPos(maxPos);
                lastPos = maxPos;
            }

            return newpath;
        }

        HexaPos GetMax(Rewards reward, PathPlanningGraph graph, HexaPath path)
        {
            int pathLen = path.Length;
            HexaPos lastPos = path[pathLen - 1];

            PlanningNode lastNode = graph[pathLen - 1].GetNode(lastPos);
            List<PlanningEdge> edges = graph[pathLen - 1].GetEdges(lastNode);
            double maxVal = -0.01;
            HexaPos maxPos = null;
            List<PlanningEdge>.Enumerator e = edges.GetEnumerator();
            while (e.MoveNext())
            {
                int nextIdx = graph[pathLen].GetIndex(e.Current.to);
                if (reward.totalRewards[pathLen][nextIdx] > maxVal)
                {
                    maxPos = graph[pathLen].mNodes[nextIdx].pos;
                    maxVal = reward.totalRewards[pathLen][nextIdx];
                }
            }

            return maxPos;
        }

        void UpdateEstimation(Rewards reward, double[,] entropy, PathPlanningGraph graph, int fromLevel, int stopAt)
        {
            for (int t = fromLevel; t <= stopAt; t++)
            {
                int num = graph[t].mNodes.Count;
                for (int i = 0; i < num; i++)
                {
                    HexaPos currentPos = graph[t].mNodes[i].pos;
                    reward.instantRewards[t][i] = GetEstimation(_agent, entropy, currentPos, _map);
                    reward.totalRewards[t][i] = reward.instantRewards[t][i];
                }
            }
        }

        void Backtrack(Rewards reward, PathPlanningGraph graph, int fromLevel, int stopAt = 0)
        {
            for (int t = fromLevel; t >= stopAt; t--)
            {
                int num = reward.totalRewards[t].Length;
                for (int i = 0; i < num; i++)
                {
                    PlanningNode node = graph[t].mNodes[i];
                    List<PlanningEdge> edges = graph[t].GetEdges(node);
                    List<PlanningEdge>.Enumerator e = edges.GetEnumerator();
                    while (e.MoveNext())
                    {
                        int j = graph[t + 1].GetIndex(e.Current.to);
                        if (reward.totalRewards[t+1][j] > reward.futureRewards[t][i])
                        {
                            reward.futureRewards[t][i] = reward.totalRewards[t+1][j];
                        }
                    }
                    reward.totalRewards[t][i] = reward.instantRewards[t][i] + reward.futureRewards[t][i];
                }
            }
        }
    }
}

