using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Hexagonal;

namespace WingmanPathPlanning.Base
{
    public class Human : Agent
    {
        int _wingmanToleranceRange;
        HexaPos _currentPos;

        public AgentMotionSequence motion;

        public int WingmanToleranceRange
        {
            get
            {
                return _wingmanToleranceRange;
            }
            
            set
            {
                _wingmanToleranceRange = value;
            }
        }

        public Human(HexagonalMap map) : base(map)
        {
            motion = new AgentMotionSequence();
            _wingmanToleranceRange = new int();            
        }

        public void SetInitPos(HexaPos pos)
        {
            _currentPos = pos;
            path.AddPos(pos);
        }

        public HexaPos GetCurrentPosition()
        {
            return _currentPos;
        }

        public void Clear()
        {
            path.Clear();
            motion.Clear();            
        }

        public void Move(HexagonalMap.Direction step)
        {
            motion.AddOneStep(step);
            _currentPos = _map.GetNext(_currentPos, step);
            path.AddPos(_currentPos);
        }
    }
}
