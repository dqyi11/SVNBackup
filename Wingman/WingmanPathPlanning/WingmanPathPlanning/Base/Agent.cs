using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Hexagonal;
using WingmanPathPlanning.Map;

namespace WingmanPathPlanning.Base
{
    public class AgentMotionSequence
    {
        public List<HexagonalMap.Direction> mMotions;

        public AgentMotionSequence()
        {
            mMotions = new List<HexagonalMap.Direction>();
        }

        public int Length
        {
            get
            {
                return mMotions.Count;
            }
        }

        public HexagonalMap.Direction this[int index]
        {
            get
            {
                return mMotions[index];
            }
        }

        public void AddOneStep(HexagonalMap.Direction step)
        {
            mMotions.Add(step);
        }

        public void Clear()
        {
            mMotions.Clear();
        }
    }

    public class Agent
    {
        const double _defaultConfidenceFactor = 0.4;
        int _observationRange;
        protected HexagonalMap _map;
        public HexaPath path;
        public int hexSetIdx;
        public double confidenceFactor;

        public Agent(HexagonalMap map)
        {
            _map = map;
            path = new HexaPath();

            confidenceFactor = _defaultConfidenceFactor;
        }

        public int GetObservationRange()
        {
            return _observationRange;
        }

        public void SetObservationRange(int range)
        {
            _observationRange = range;
        }

        public HexagonalMap GetMap()
        {
            return _map;
        }

        public HexaPath ConvertToPath(AgentMotionSequence sequence, HexaPos currentPos)
        {
            HexagonalMap map = GetMap();
            HexaPath path = new HexaPath();

            path.AddPos(currentPos);

            List<HexagonalMap.Direction>.Enumerator e = sequence.mMotions.GetEnumerator();

            while (e.MoveNext())
            {
                HexagonalMap.Direction step = e.Current;
                currentPos = map.GetNext(currentPos, step);
                path.AddPos(currentPos);
            }

            return path;
        }

        public AgentMotionSequence ConvertToMotionSequence(HexaPath path)
        {
            AgentMotionSequence motions = new AgentMotionSequence();

            return motions;
        }

        public double Update(HexaPath path, double[,] entropy)
        {
            double deltaScore = 0.0;
            double deltaStepScore = 0.0;
            List<HexaPos> neighbors;
            List<HexaPos>.Enumerator e;

            for (int t = 0; t < path.Length; t++)
            {
                deltaStepScore = entropy[path[t].X, path[t].Y];
                entropy[path[t].X, path[t].Y] -= deltaStepScore;
                deltaScore += deltaStepScore;

                neighbors = _map.GetHexes(path[t].X, path[t].Y, GetObservationRange());
                e = neighbors.GetEnumerator();
                while (e.MoveNext())
                {
                    deltaStepScore = confidenceFactor * entropy[e.Current.X, e.Current.Y];
                    deltaScore += deltaStepScore;
                    entropy[e.Current.X, e.Current.Y] -= deltaStepScore;
                }
            }

            return deltaScore;
        }

        public double Score(HexaPath path, double[,] entropy)
        {
            double[,] localEntropy = (double[,])(entropy.Clone());
            return Update(path, localEntropy);
        }
    }
}
