using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Hexagonal;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Base;

namespace WingmanPathPlanning.Planner
{
    public class ExhaustiveDFSPathPlanner :  PathPlanner 
    {
        HexaPath _maxPath;
        double _maxScore;
        int _pathNumCnt;

        public ExhaustiveDFSPathPlanner(HexagonalMap map, Robot agent)
            : base(map, agent)
        {
            _maxPath = null;
            _maxScore = 0.0;
            _pathNumCnt = 0;
        }

        public override HexaPath FindPath(PathPlanningGraph graph, HexaPos start)
        {
            _maxPath = null;
            _maxScore = 0.0;
            _pathNumCnt = 0;

            HexaPath path = new HexaPath();

            //path.AddPos(start);

            FindMaxPath(graph, path, start);

            return _maxPath;
        }

        double ScorePath(HexaPath path)
        {
            return _agent.Score(path, _localEntropy);            
        }

        void FindMaxPath(PathPlanningGraph graph, HexaPath path, HexaPos newPos)
        {
            HexaPath currentPath = path.Clone();
            currentPath.AddPos(newPos);
            // Console.WriteLine(currentPath.ToString());
            if (graph.planningLength == currentPath.Length)
            {
                double newScore = ScorePath(currentPath);
                _pathNumCnt++;

                if (_maxPath == null)
                {
                    _maxPath = currentPath;
                    _maxScore = newScore;
                }
                else
                {
                    if (newScore > _maxScore)
                    {
                        _maxPath = currentPath;
                        _maxScore = newScore;
                    }
                }

                //Console.WriteLine("number " + _pathNumCnt + ", new score " + newScore + ", max score "+  _maxScore);
                
                return;                
            }

            //Console.WriteLine(graph.planningLength + " - " + currentPath.Length);

            List<PlanningNode>.Enumerator e = graph[currentPath.Length].mNodes.GetEnumerator();
            while (e.MoveNext())
            {
                if (_map.IsAccessible(currentPath[currentPath.Length - 1], e.Current.pos) == true)
                {
                    //currentPath.AddPos(e.Current.pos);
                    FindMaxPath(graph, currentPath, e.Current.pos);
                }
            }
        }
    }
}
