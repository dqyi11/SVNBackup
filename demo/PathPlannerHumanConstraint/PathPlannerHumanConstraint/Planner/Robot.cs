using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using PathPlanner.Hexagonal;

namespace PathPlanner.Planner
{
    public class Robot : Agent
    {
        HexaPos _currentPos;

        public Robot(HexagonalMap map)
            : base(map)
        {

        }
    }
}
