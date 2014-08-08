using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Xml;

namespace MapLabeling.data
{
    public class MapInfoManager
    {
        public enum LabelType { NONE = 0, FEATURE = 1, OUTDOOR = 2, INDOOR = 3, ENEMY = 4 };
        public FeatureLabelManager featureMgr = null;
        public IndoorLabelManager indoorMgr = null;
        public OutdoorLabelManager outdoorMgr = null;
        public EnemyLabelManager enemyMgr = null;

        public string mapFilename = null;
        public string worldFilename = null;
        public string obstacleFilename = null;
        public int mapWidth;
        public int mapHeight;

        public int activeX;
        public int activeY;

        public MapInfoManager()
        {
            featureMgr = new FeatureLabelManager(this);
            indoorMgr = new IndoorLabelManager(this);
            outdoorMgr = new OutdoorLabelManager(this);
            enemyMgr = new EnemyLabelManager(this);

            mapFilename = "";
            worldFilename = "";
            obstacleFilename = "";

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
                    enemyMgr.ResetActiveEnemy();
                    indoorMgr.ResetActiveIndoor();
                    outdoorMgr.ResetActiveOutdoor();
                }
            }

            if (isFound == false)
            {
                isFound = enemyMgr.FindActiveEnemy(x, y);
                if (isFound == true)
                {
                    activeLabelType = LabelType.ENEMY;
                    featureMgr.ResetActiveFeature();
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
                    enemyMgr.ResetActiveEnemy();
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
                    enemyMgr.ResetActiveEnemy();
                    indoorMgr.ResetActiveIndoor();
                }
            }

            if (isFound == false)
            {
                activeLabelType = LabelType.NONE;
                featureMgr.ResetActiveFeature();
                enemyMgr.ResetActiveEnemy();
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
            else if (activeLabelType == LabelType.ENEMY)
            {
                enemyMgr.DeleteActiveFeature();
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
                xtw.WriteAttributeString("ObstacleFile", this.obstacleFilename);
                xtw.WriteAttributeString("MapWidth", this.mapWidth.ToString());
                xtw.WriteAttributeString("MapHeight", this.mapHeight.ToString());

                xtw.WriteStartElement("features");
                xtw.WriteRaw(this.featureMgr.DumpToString());
                xtw.WriteEndElement();

                xtw.WriteStartElement("indoors");
                xtw.WriteRaw(this.indoorMgr.DumpToString());
                xtw.WriteEndElement();

                xtw.WriteStartElement("outdoors");
                xtw.WriteRaw(this.outdoorMgr.DumpToString());
                xtw.WriteEndElement();

                xtw.WriteStartElement("enemies");
                xtw.WriteRaw(this.enemyMgr.DumpToString());
                xtw.WriteEndElement();

                xtw.WriteEndElement();
                xtw.WriteEndDocument();
            }
        }

        public void LoadFile(string filename)
        {

            System.Xml.XmlDocument xmlDoc = new System.Xml.XmlDocument();
            xmlDoc.Load(filename); //Loads the document
            System.Xml.XmlNodeList nodeList = xmlDoc.GetElementsByTagName("MapLabel");
            System.Xml.XmlNode currentNode = nodeList[0];

            this.mapFilename = currentNode.Attributes["MapFile"].Value;
            this.worldFilename = currentNode.Attributes["WorldFile"].Value;
            this.obstacleFilename = currentNode.Attributes["ObstacleFile"].Value;
            this.mapWidth = Int32.Parse(currentNode.Attributes["MapWidth"].Value);
            this.mapHeight = Int32.Parse(currentNode.Attributes["MapHeight"].Value);

            System.Xml.XmlNode featureNode = xmlDoc.SelectNodes("/MapLabel/features")[0];
            string featureStr = featureNode.InnerXml;
            if (featureStr != "")
            {
                string newFeatureStr = "<features>";
                newFeatureStr += featureStr;
                newFeatureStr += "</features>";
                featureMgr.LoadFromString(newFeatureStr);
            }
            System.Xml.XmlNode indoorNode = xmlDoc.SelectNodes("/MapLabel/indoors")[0];
            string indoorStr = indoorNode.InnerXml;
            if (indoorStr != "")
            {
                string newIndoorStr = "<indoors>";
                newIndoorStr += indoorStr;
                newIndoorStr += "</indoors>";
                indoorMgr.LoadFromString(newIndoorStr);
            }
            System.Xml.XmlNode outdoorNode = xmlDoc.SelectNodes("/MapLabel/outdoors")[0];
            string outdoorStr = outdoorNode.InnerXml;
            if (outdoorStr != "")
            {
                string newOutdoorStr = "<indoors>";
                newOutdoorStr += outdoorStr;
                newOutdoorStr += "</indoors>";
                outdoorMgr.LoadFromString(newOutdoorStr);
            }
            System.Xml.XmlNode enemyNode = xmlDoc.SelectNodes("/MapLabel/enemies")[0];
            string enemyStr = enemyNode.InnerXml;
            if (enemyStr != "")
            {
                string newEnemyStr = "<enemies>";
                newEnemyStr += enemyStr;
                newEnemyStr += "</enemies>";
                enemyMgr.LoadFromString(newEnemyStr);
            }

            /*
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
                            string featureStr = "<features>";
                            featureStr += xtr.ReadElementContentAsString();
                            featureStr = featureStr + "</features>";
                            featureMgr.LoadFromString(featureStr);
                            
                        }
                        else if (xtr.Name == "indoors")
                        {
                            string indoorStr = "<indoors>";
                            indoorStr += xtr.ReadElementContentAsString();
                            indoorStr = indoorStr + "</indoors>";
                            indoorMgr.LoadFromString(indoorStr);                            
                        }
                        else if (xtr.Name == "outdoors")
                        {
                            string outdoorStr = "<outdoors>";
                            outdoorStr += xtr.ReadElementContentAsString();
                            outdoorStr = outdoorStr + "</outdoors>";
                            outdoorMgr.LoadFromString(outdoorStr);
                        }
                    }
                }
            }
             * */
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
