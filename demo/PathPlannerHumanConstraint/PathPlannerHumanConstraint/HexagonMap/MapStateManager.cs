using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using PathPlanner.Planner;

namespace PathPlanner.Hexagonal
{
    public class MapStateManager
    {
        const double _initProb = 0.5;
        HexagonalMap _map;
        int _width;
        int _height;
        double[,] _probability;
        double[,] _entropy;

        public MapStateManager(HexagonalMap map)
        {
            _map = map;
            _width = _map.mapWidth;
            _height = _map.mapHeight;
            _probability = new double[_width, _height];
            _entropy = new double[_width, _height];
            Init();
        }

        public MapStateManager(MapStateManager baseMgr)
        {
            _map = baseMgr._map;
            _width = baseMgr._width;
            _height = baseMgr._height;
            _probability = (double[,])baseMgr._probability.Clone();
            _entropy = (double[,])baseMgr._entropy.Clone();
        }

        void Init()
        {
            for (int i = 0; i < _width; i++)
            {
                for (int j = 0; j < _height; j++)
                {
                    SetProbability(i, j, _initProb);
                }
            }
        }

        public double[,] CopyEntropy()
        {
            return (double[,])_entropy.Clone();
        }

        public void RandomizeValue()
        {
            Random rnd = new Random();
            for (int i = 0; i < _width; i++)
            {
                for (int j = 0; j < _height; j++)
                {
                    SetProbability(i, j, rnd.NextDouble());
                }
            }
        }

        public void Reset()
        {
            Init();
        }

        public void SetProbability(int x, int y, double prob)
        {
            _probability[x, y] = prob;
            _entropy[x, y] = CalcEntropy(prob);
        }

        public void SetEntropy(int x, int y, double val)
        {
            _entropy[x, y] = val;
        }

        public double CalcEntropy(double prob)
        {
            double entropy = 0.0;

            if (prob == 1 || prob == 0)
            {
                return entropy;
            }

            entropy = prob * Math.Log(prob,2) + (1 - prob) * Math.Log(1 - prob,2);
            entropy = -entropy;

            return entropy;
        }

        public double[,] GetEntropy()
        {
            return _entropy;
        }

        public double GetEntropy(int x, int y)
        {
            return _entropy[x, y];
        }

        public Color GetHexColor(int x, int y)
        {
            double val = GetEntropy(x, y);
            return GetColor(val);
        }

        public Color GetRGBColor(int x, int y)
        {
            double val = GetEntropy(x, y);
            return GetRGBColor(val);
        }

        public Color GetRGBColor(double value)
        {
            Color color;

            int red;
            int green;
            int blue;
            ColorConverter.HsvToRgb(value * 360, 1, 1, out red, out green, out blue);
            //red = (int)(value * 255);
            //green = (int)(value * 255);
            //blue = (int)(value * 255);

            color = Color.FromArgb(red, green, blue);

            return color;
        }

        public Color GetColor(double value)
        {
            int gray = (int)(255.0 * value);
            return Color.FromArgb(gray, gray, gray);
        }

        public Color GetProbabilityColor(int x, int y)
        {
            int gray = (int)(255.0 * _probability[x,y]);
            return Color.FromArgb(gray, gray, gray);
        }

        public HexaPos GetMaxPos(List<HexaPos> hexes)
        {
            double maxValue = 0;
            HexaPos maxPos = null;

            List<HexaPos>.Enumerator e = hexes.GetEnumerator();
            while (e.MoveNext())
            {
                if (_entropy[e.Current.X, e.Current.Y] >= maxValue)
                {
                    maxPos = e.Current;
                }
            }

            return maxPos;
        }

        public void Update(Agent agent, HexaPos currentPos)
        {
            int radius = agent.GetObservationRange();

            SetProbability(currentPos.X, currentPos.Y, 0);

            List<HexaPos> hexes = _map.GetHexes(currentPos.X, currentPos.Y, radius, false);

            List<HexaPos>.Enumerator e = hexes.GetEnumerator();

            while (e.MoveNext())
            {
                double prob = _probability[currentPos.X, currentPos.Y];
                prob = prob * agent.confidenceFactor;
                SetProbability(currentPos.X, currentPos.Y, prob);
            }
        }

        public void Update(Agent agent, HexaPath path)
        {
            /*
            for (int t = 0; t < path.Length; t++)
            {
                Update(agent, path[t]);
            }  
             */
            agent.Update(path, _entropy);
        }
    }
}
