using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Hexagonal;

namespace WingmanPathPlanning.Base
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
