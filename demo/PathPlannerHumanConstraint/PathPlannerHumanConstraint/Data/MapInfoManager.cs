using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Xml;

namespace PathPlanner.Data
{
    public class MapInfoManager
    {
        public enum LabelType { NONE = 0, FEATURE = 1, OUTDOOR = 2, INDOOR = 3 };
        public FeatureLabelManager featureMgr = null;
        public IndoorLabelManager indoorMgr = null;
        public OutdoorLabelManager outdoorMgr = null;

        public string mapFilename = null;
        public string worldFilename = null;
        public int mapWidth;
        public int mapHeight;

        public int activeX;
        public int activeY;

        public MapInfoManager()
        {
            featureMgr = new FeatureLabelManager(this);
            indoorMgr = new IndoorLabelManager(this);
            outdoorMgr = new OutdoorLabelManager(this);

            mapFilename = "";
            worldFilename = "";

            activeX = 0;
            activeY = 0;
        }

        public LabelType activeLabelType = LabelType.NONE;

        public void UpdateActiveLabel(int x, int y)
        {
            activeLabelType = LabelType.NONE;
            bool isFound = false;

            if (isFound == false)
            {
                isFound = featureMgr.FindActiveFeature(x, y);
                if (isFound == true)
                {
                    activeLabelType = LabelType.FEATURE;
                    indoorMgr.ResetActiveIndoor();
                    outdoorMgr.ResetActiveOutdoor();
                }
            }

            if (isFound == false)
            {
                isFound = indoorMgr.FindActiveIndoor(x, y);
                if (isFound == true)
                {
                    activeLabelType = LabelType.INDOOR;
                    featureMgr.ResetActiveFeature();
                    outdoorMgr.ResetActiveOutdoor();
                }
            }

            if (isFound == false)
            {
                isFound = outdoorMgr.FindActiveOutdoor(x, y);
                if (isFound == true)
                {
                    activeLabelType = LabelType.OUTDOOR;
                    featureMgr.ResetActiveFeature();
                    indoorMgr.ResetActiveIndoor();
                }
            }

            if (isFound == false)
            {
                activeLabelType = LabelType.NONE;
                featureMgr.ResetActiveFeature();
                indoorMgr.ResetActiveIndoor();
                outdoorMgr.ResetActiveOutdoor();
            }
        }


        public void DeleteActiveLabel()
        {
            if (activeLabelType == LabelType.FEATURE)
            {
                featureMgr.DeleteActiveFeature();
                activeLabelType = LabelType.NONE;
            }
            else if (activeLabelType == LabelType.INDOOR)
            {
                indoorMgr.DeleteActiveIndoor();
                activeLabelType = LabelType.NONE;
            }
            else if (activeLabelType == LabelType.OUTDOOR)
            {
                outdoorMgr.DeleteActiveOutdoor();
                activeLabelType = LabelType.NONE;
            }
        }

        public void DumpToFile(string filename)
        {
            using (StreamWriter sw = new StreamWriter(filename))
            {
                XmlTextWriter xtw = new XmlTextWriter(sw);
                xtw.WriteStartDocument();
                xtw.WriteStartElement("MapLabel");
                xtw.WriteAttributeString("MapFile", this.mapFilename);
                xtw.WriteAttributeString("WorldFile", this.worldFilename);
                xtw.WriteAttributeString("MapWidth", this.mapWidth.ToString());
                xtw.WriteAttributeString("MapHeight", this.mapHeight.ToString());

                xtw.WriteStartElement("features");
                xtw.WriteString(this.featureMgr.DumpToString());
                xtw.WriteEndElement();

                xtw.WriteStartElement("indoors");
                xtw.WriteString(this.indoorMgr.DumpToString());
                xtw.WriteEndElement();

                xtw.WriteStartElement("outdoors");
                xtw.WriteString(this.outdoorMgr.DumpToString());
                xtw.WriteEndElement();

                xtw.WriteEndElement();
                xtw.WriteEndDocument();
            }
        }

        public void LoadFile(string filename)
        {
            using (StreamReader sr = new StreamReader(filename))
            {
                XmlTextReader xtr = new XmlTextReader(sr);

                while (xtr.Read())
                {
                    if (xtr.NodeType == XmlNodeType.Element)
                    {
                        if (xtr.Name == "MapLabel")
                        {
                            this.mapFilename = xtr.GetAttribute("MapFile");
                            this.worldFilename = xtr.GetAttribute("WorldFile");
                            this.mapWidth = Int32.Parse(xtr.GetAttribute("MapWidth"));
                            this.mapHeight = Int32.Parse(xtr.GetAttribute("MapHeight"));
                            
                        }
                        else if (xtr.Name == "features")
                        {
                            string featureStr = xtr.ReadString();
                            featureStr = "<features>" + featureStr;
                            featureStr = featureStr + "</features>";
                            featureMgr.LoadFromString(featureStr);
                            
                        }
                        else if (xtr.Name == "indoors")
                        {
                            string indoorStr = xtr.ReadString();
                            indoorStr = "<indoors>" + indoorStr;
                            indoorStr = indoorStr + "</indoors>";
                            indoorMgr.LoadFromString(indoorStr);                            
                        }
                        else if (xtr.Name == "outdoors")
                        {
                            string outdoorStr = xtr.ReadString();
                            outdoorStr = "<outdoors>" + outdoorStr;
                            outdoorStr = outdoorStr + "</outdoors>";
                            outdoorMgr.LoadFromString(outdoorStr);
                        }
                    }
                }
            }
        }

        public string GetMapFilename()
        {
            return mapFilename;
        }

        public double GetDiffusionValue(int x, int y)
        {
            double value = 0.0;
            if (x < 0 && x >= this.mapWidth)
            {
                return value;
            }
            if (y < 0 && y >= this.mapHeight)
            {
                return value;
            }
            for (int i = 0; i < this.outdoorMgr.outdoors.Count; i++)
            {
                OutdoorLabel label = this.outdoorMgr.outdoors[i];

                if (label.type == OutdoorLabel.OUTDOOR_TYPE.MARKET)
                {
                    double xDist = (double)(x - label.center.X);
                    double yDist = (double)(y - label.center.Y);
                    double dist = Math.Pow(xDist, 2) + Math.Pow(yDist, 2);
                    double newVal = Math.Exp(-dist);
                    if (newVal > value)
                    {
                        value = newVal;
                    }
                }

            }

            return value;
        }

    }
}
