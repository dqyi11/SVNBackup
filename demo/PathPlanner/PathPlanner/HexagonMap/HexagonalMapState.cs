using System;
using System.Collections.Generic;
using System.Text;
using System.Drawing;

namespace PathPlanner.Hexagonal
{

    public class HexSet
    {
        public System.Drawing.Color gridColor;
        public System.Drawing.Color borderColor;

        public List<Hex> hexSet;

        public HexSet(HexagonalMapState mapState)
        {
            gridColor = mapState.GridColor;
            borderColor = mapState.ActiveHexBorderColor;

            hexSet = new List<Hex>();
        }

        public void Clear()
        {
            hexSet.Clear();
        }
    }

	public class HexagonalMapState
	{
		private System.Drawing.Color backgroundColor;
		private System.Drawing.Color gridColor;
		private int gridPenWidth;
		private Hex activeHex;
        private Hex activeHex2;
		private System.Drawing.Color activeHexBorderColor;
        private System.Drawing.Color activeHexBorderColor2;
		private int activeHexBorderWidth;

        private int mapWidth;
        private int mapHeight;

        public double[,] hexVals;

        List<HexSet> sets;

        public HexSet obstacles;

        HexSet activeSet;

		#region Properties

		public System.Drawing.Color BackgroundColor
		{
			get
			{
				return backgroundColor;
			}
			set
			{
				backgroundColor = value;
			}
		}

		public System.Drawing.Color GridColor
		{
			get
			{
				return gridColor;
			}
			set
			{
				gridColor = value;
			}
		}

		public int GridPenWidth
		{
			get
			{
				return gridPenWidth;
			}
			set
			{
				gridPenWidth = value;
			}
		}

        public HexSet ActiveHexSet
        {
            get
            {
                return activeSet;
            }

            set
            {
                activeSet = value;
            }
        }

		public Hex ActiveHex
		{
			get
			{
				return activeHex;
			}
			set
			{
				activeHex = value;
			}
		}

        public Hex ActiveHex2
        {
            get
            {
                return activeHex2;
            }
            set
            {
                activeHex2 = value;
            }
        }

		public System.Drawing.Color ActiveHexBorderColor
		{
			get
			{
				return activeHexBorderColor;
			}
			set
			{
				activeHexBorderColor = value;
			}
		}

        public System.Drawing.Color ActiveHexBorderColor2
        {
            get
            {
                return activeHexBorderColor2;
            }
            set
            {
                activeHexBorderColor2 = value;
            }
        }

		public int ActiveHexBorderWidth
		{
			get
			{
				return activeHexBorderWidth;
			}
			set
			{
				activeHexBorderWidth = value;
			}
		}
		#endregion

		public HexagonalMapState(int width, int height)
		{
			backgroundColor = Color.White;
			gridColor = Color.Black;
			gridPenWidth = 1;
			activeHex = null;
            activeHex2 = null;
			activeHexBorderColor = Color.Blue;
            activeHexBorderColor2 = Color.Green;
			activeHexBorderWidth = 3;

            mapWidth = width;
            mapHeight = height;
            hexVals = new double[width, height];

            sets = new List<HexSet>();
            obstacles = new HexSet(this);
            activeSet = null;
		}

        public void ClearSet()
        {
            sets.Clear();
        }

        public int CreateHexSet()
        {
            int currentIdx = sets.Count;

            HexSet set = new HexSet(this);
            AddHexSet(set);
            return currentIdx;
        }

        public void AddHexSet(HexSet set)
        {
            sets.Add(set);
        }


        public HexSet GetHexSet(int index)
        {
            if (sets.Count == 0)
            {
                return null;
            }
            if (index >= sets.Count || index < 0)
            {
                return null;
            }
            return sets[index];            
        }

        public int GetHexSetNum()
        {
            return sets.Count;
        }

        public void AddObstalce(Hex obstacle)
        {
            obstacles.hexSet.Add(obstacle);
        }

        public void ClearObstacle()
        {
            obstacles.hexSet.Clear();
        }

        public bool IsObstacle(Hex pos)
        {
            List<Hex>.Enumerator e = obstacles.hexSet.GetEnumerator();
            while (e.MoveNext())
            {
                if (e.Current == pos)
                {
                    return true;
                }
            }

            return false;
        }

        public void DumpHexValToFile(string filename)
        {
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(filename))
            {
                for (int i = 0; i < this.mapHeight; i++)
                {
                    string line = "";
                    for (int j = 0; j < this.mapWidth; j++)
                    {
                        line = line + this.hexVals[j, i].ToString() +", "; 
                    }
                    file.WriteLine(line);
                }
            }
        }

        public void NormalizeHexVal()
        {
            double minVal = this.hexVals[0, 0];
            double maxVal = this.hexVals[0, 0];

            for (int i = 0; i < this.mapWidth; i++)
            {
                for (int j = 0; j < this.mapHeight; j++)
                {
                    if (this.hexVals[i, j] < minVal)
                    {
                        minVal = this.hexVals[i, j];
                    }
                    if (this.hexVals[i, j] > maxVal)
                    {
                        maxVal = this.hexVals[i, j];
                    }
                }
            }

            for (int i = 0; i < this.mapWidth; i++)
            {
                for (int j = 0; j < this.mapHeight; j++)
                {
                    double val = this.hexVals[i, j];
                    val = (val - minVal) / (maxVal - minVal);
                    this.hexVals[i, j] = val;
                }
            }

        }
	}
}