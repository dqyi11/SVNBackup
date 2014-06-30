using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using WingmanPathPlanning.Hexagonal;

namespace WingmanPathPlanning.Map
{
    class VisibilityGraph
    {
        HexagonalMap _visibilityGraph;
        HexagonalMap _map;

        int _width;
        int _height;

        public HexagonalMap VisGraph
        {
            get
            {
                return _visibilityGraph;
            }
        }

        public VisibilityGraph(HexagonalMap map)
        {
            _map = map;
            _width = _map.mapWidth;
            _height = _map.mapHeight;
            
            _visibilityGraph = new HexagonalMap(_width, _height, _map.mapSide/2, _map.mapOrientation);
        }
    }
}
