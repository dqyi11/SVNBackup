using System;
using System.Collections.Generic;
using System.Text;
using System.Drawing;

namespace WingmanPathPlanning.Hexagonal
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
		private Hexagonal.Hex activeHex;
		private System.Drawing.Color activeHexBorderColor;
		private int activeHexBorderWidth;

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

		public Hexagonal.Hex ActiveHex
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

		public HexagonalMapState()
		{
			backgroundColor = Color.White;
			gridColor = Color.Black;
			gridPenWidth = 1;
			activeHex = null;
			activeHexBorderColor = Color.Blue;
			activeHexBorderWidth = 1;

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
	}
}