using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Xml;

namespace WingmanPathPlanning.Data
{
    public class ParameterManager
    {
        const string _defaultFilename = @"setting.xml";
        const int _defaultWinFormWidth = 800;
        const int _defaultWinFormHeight = 600;

        const int _defaultHexagonalWidthNum = 26;
        const int _defaultHexagonalHeightNum = 26;
        const int _defaultHexagonalSize = 20;

        const int _defaultSimMapSize = 15;
        const int _defaultSimMapWidth = 6;
        const int _defaultSimMapHeight = 6;

        const int _defaultWingmanRadius = 2;
        const int _defaultHumanObservation = 2;
        const int _defaultRobotObservation = 2;

        const double _defaultHumanObservationFactor = 0.4;
        const double _defaultRobotObservationFactor = 0.4;

        int _winFormWidth;
        int _winFormHeight;
        int _hexagonalWidthNum;
        int _hexagonalHeightNum;
        int _hexagonalSize;

        int _wingmanRadius;
        int _humanObservation;
        int _robotObservation;

        double _humanObservationFactor;
        double _robotObservationFactor;

        int _simTestWidth;
        int _simTestHeight;
        int _simTestSize;


        protected XmlDocument _xDoc;

        public int winFormWidth
        {
            get
            {
                return _winFormWidth;
            }
        }

        public int winFormHeight
        {
            get
            {
                return _winFormHeight;
            }
        }

        public int hexagonalWidthNum
        {
            get
            {
                return _hexagonalWidthNum;
            }
        }

        public int hexagonalHeightNum
        {
            get
            {
                return _hexagonalHeightNum;
            }
        }

        public int hexagonalSize
        {
            get
            {
                return _hexagonalSize;
            }
        }

        public int wingmanRadius
        {
            get
            {
                return _wingmanRadius;
            }

            set
            {
                _wingmanRadius = value;
            }
        }

        public int humanObservation
        {
            get
            {
                return _humanObservation;
            }

            set
            {
                _humanObservation = value;
            }
        }

        public int robotObservation
        {
            get
            {
                return _robotObservation;
            }

            set
            {
                _robotObservation = value;
            }
        }

        public double humanObservationFactor
        {
            get
            {
                return _humanObservationFactor;
            }

            set
            {
                _humanObservationFactor = value;
            }
        }

        public double robotObservationFactor
        {
            get
            {
                return _robotObservationFactor;
            }

            set
            {
                _robotObservationFactor = value;
            }
        }

        public int simTestWidth
        {
            get
            {
                return _simTestWidth;
            }
        }

        public int simTestHeight
        {
            get
            {
                return _simTestHeight;
            }
        }

        public int simTestSize
        {
            get
            {
                return _simTestSize;
            }
        }

        public ParameterManager()
        {
            _winFormWidth = _defaultWinFormWidth;
            _winFormHeight = _defaultWinFormHeight;
            _hexagonalWidthNum = _defaultHexagonalWidthNum;
            _hexagonalHeightNum = _defaultHexagonalHeightNum;
            _hexagonalSize = _defaultHexagonalSize;

            _wingmanRadius = _defaultWingmanRadius;
            _humanObservation = _defaultHumanObservation;
            _robotObservation = _defaultRobotObservation;

            _humanObservationFactor = _defaultHumanObservationFactor;
            _robotObservationFactor = _defaultRobotObservationFactor;

            _simTestWidth = _defaultSimMapWidth;
            _simTestHeight = _defaultSimMapHeight;
            _simTestSize = _defaultSimMapSize;

            _xDoc = new XmlDocument();
            
        }

        public void Load()
        {
            _xDoc.Load(_defaultFilename);

            XmlNode settingNode = _xDoc.SelectSingleNode("Setting");

            XmlNode formNode = settingNode.SelectSingleNode("Form");
            XmlNode formWidth = formNode.Attributes.GetNamedItem("width");
            XmlNode formHeight = formNode.Attributes.GetNamedItem("height");

            if (formWidth != null)
            {
                _winFormWidth = int.Parse(formWidth.Value);
            }
            if (formHeight != null)
            {
                _winFormHeight = int.Parse(formHeight.Value);
            }

            XmlNode mapNode = settingNode.SelectSingleNode("HexaMap");
            XmlNode mapWidth = mapNode.Attributes.GetNamedItem("width");
            XmlNode mapHeight = mapNode.Attributes.GetNamedItem("height");
            XmlNode mapSize = mapNode.Attributes.GetNamedItem("size");

            if (mapWidth != null)
            {
                _hexagonalWidthNum = int.Parse(mapWidth.Value);
            }
            if (mapHeight != null)
            {
                _hexagonalHeightNum = int.Parse(mapHeight.Value);
            }
            if (mapSize != null)
            {
                _hexagonalSize = int.Parse(mapSize.Value);
            }

            XmlNode simMapNode = settingNode.SelectSingleNode("SimHexaMap");
            XmlNode simMapWidth = simMapNode.Attributes.GetNamedItem("width");
            XmlNode simMapHeight = simMapNode.Attributes.GetNamedItem("height");
            XmlNode simMapSize = simMapNode.Attributes.GetNamedItem("size");

            if (mapWidth != null)
            {
                _simTestWidth = int.Parse(simMapWidth.Value);
            }
            if (mapHeight != null)
            {
                _simTestHeight = int.Parse(simMapHeight.Value);
            }
            if (mapSize != null)
            {
                _simTestSize = int.Parse(simMapSize.Value);
            }
        }      
    }
}
