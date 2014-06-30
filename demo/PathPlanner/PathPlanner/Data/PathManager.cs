using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Xml;
using PathPlanner.Hexagonal;

namespace PathPlanner.Data
{
    class PathManager
    {
        HexagonalMap _map;
        string _mapFilename;

        public PathManager(HexagonalMap map, string map_filename)
        {
            _map = map;
            _mapFilename = map_filename;
        }

        public void DumpPath(HexaPath path, string filename)
        {
            using (StreamWriter sw = new StreamWriter(filename))
            {
                XmlTextWriter xtw = new XmlTextWriter(sw);
                xtw.WriteStartDocument();
                xtw.WriteStartElement("path");
                xtw.WriteAttributeString("map", _mapFilename);

                for (int t = 0; t < path.Length; t++)
                {
                    HexaPos pos = path[t];
                    xtw.WriteStartElement("position");
                    xtw.WriteAttributeString("idx_x", pos.X.ToString());
                    xtw.WriteAttributeString("idx_y", pos.Y.ToString());
                    xtw.WriteAttributeString("pos_x", _map.GetHex(pos.X, pos.Y).centroid.X.ToString());
                    xtw.WriteAttributeString("pos_y", _map.GetHex(pos.X, pos.Y).centroid.Y.ToString());
                    xtw.WriteString(t.ToString());
                    xtw.WriteEndElement();
                }

                xtw.WriteEndElement();
                xtw.WriteEndDocument();
            }
        }
    }
}
