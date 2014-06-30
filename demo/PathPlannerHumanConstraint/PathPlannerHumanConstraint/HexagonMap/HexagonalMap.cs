using System;
using System.Collections.Generic;
using System.Text;
using System.Drawing;

namespace PathPlanner.Hexagonal
{
    public class HexaPos
    {
        public HexaPos(int x, int y)
        {
            _x = x;
            _y = y;
        }
        public int X
        {
            get
            {
                return _x;
            }
        }
        public int Y
        {
            get
            {
                return _y;
            }
        }

        public override bool Equals(object obj)
        {
            if (this._x == ((HexaPos)obj)._x
                && this._y == ((HexaPos)obj)._y)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        public bool Equals(HexaPos pos)
        {
            if (this._x == pos._x && this._y == pos._y)
            {
                return true;
            }

            return false;
        }

        public string GetName()
        {
            return Convert.ToString(_x) + "-" + Convert.ToString(_y);
        }

        private int _x;
        private int _y;
    }

    public class HexaPath
    {
        List<HexaPos> _path;

        public int Length
        {
            get
            {
                return _path.Count;
            }
        }

        public HexaPath()
        {
            _path = new List<HexaPos>();
        }

        public HexaPath(HexaPath other)
        {
            _path = new List<HexaPos>();
            List<HexaPos>.Enumerator e = other._path.GetEnumerator();
            while (e.MoveNext())
            {
                AddPos(e.Current);
            }
        }

        public void AddPos(HexaPos pos)
        {
            _path.Add(pos);
        }

        public void InsertFront(HexaPos pos)
        {
            _path.Insert(0, pos);
        }

        public void Merge(HexaPath other)
        {
            int otherLen = other.Length;
            for (int t = 0; t < otherLen; t++)
            {
                _path.Add(other[t]);
            }
        }

        public HexaPos this[int index]
        {
            get
            {
                return _path[index];
            }
        }

        public void Clear()
        {
            _path.Clear();
        }

        public HexaPath Clone()
        {
            HexaPath newPath = new HexaPath();
            List<HexaPos>.Enumerator e = _path.GetEnumerator();
            while (e.MoveNext())
            {
                newPath.AddPos(e.Current);
            }

            return newPath;
        }

        public virtual string ToString()
        {
            string output = "HexaPath: ";
            List<HexaPos>.Enumerator e = _path.GetEnumerator();
            while (e.MoveNext())
            {
                output += e.Current.GetName() +" ";
            }

            return output;
        }

        public int DifferentAt(HexaPath other)
        {
            int selfLen = Length;
            int otherLen = other.Length;
            for (int t = 0; t < selfLen && t < otherLen; t++)
            {
                if (_path[t].X != other._path[t].X
                    || _path[t].Y != other._path[t].Y)
                {
                    return t;
                }
            }
            return selfLen;
        }

        public HexaPath SubPath(int from, int to)
        {
            HexaPath subpath = new HexaPath();
            for (int t = from; t <= to; t++)
            {
                subpath.AddPos(_path[t]);
            }
            return subpath;
        }
    }

	/// <summary>
	/// Represents a 2D hexagon board
	/// </summary>
	public class HexagonalMap
	{
		private Hex[,] hexes;
		private int width;
		private int height;
		private int xOffset;
		private int yOffset;
		private int side;
		private float pixelWidth;
		private float pixelHeight;
		private HexOrientation orientation;
		private System.Drawing.Color backgroundColor;
		public HexagonalMapState mapState;
        private MapStateManager mapStateMgr;
        
        public enum Direction { EAST, WEST, NE, NW, SE, SW, STAY };

        public Direction[] directionList = { Direction.EAST, Direction.WEST, 
                                        Direction.NE, Direction.NW, 
                                        Direction.SE, Direction.SW,
                                        Direction.STAY };

        public int mapWidth
        {
            get
            {
                return width;
            }
        }

        public int mapHeight
        {
            get
            {
                return height;
            }
        }

        public int mapSide
        {
            get
            {
                return side;
            }
        }

        public HexOrientation mapOrientation
        {
            get
            {
                return orientation;
            }
        }

        public Hex GetHex(int x, int y)
        {
            return hexes[y, x];
        }

        public List<HexaPos> GetHexes(int x, int y, int radius, bool includeCenter = false)
        {
            List<HexaPos> hexes = new List<HexaPos>();
            List<HexaPos> delta = new List<HexaPos>();
            HexaPos centerHexPos = new HexaPos(x, y);
        
            hexes.Add(centerHexPos);

            if (radius < 1)
            {
                return hexes;
            }

            for (int i = 0; i < radius; i++)
            {
                List<HexaPos>.Enumerator e = hexes.GetEnumerator();

                delta.Clear();
                
                while (e.MoveNext())
                {
                    int refX = e.Current.X;
                    int refY = e.Current.Y;

                    for (int j = 0; j < directionList.Length; j++)
                    {
                        HexaPos next = GetNext(refX, refY, directionList[j]);

                        //Console.WriteLine(refX + " + " + refY + " -- " + directionList[j].ToString() + " = " + next.X + " + " + next.Y);

                        if (!hexes.Contains(next))
                        {
                            delta.Add(next);
                        }
                    }
                }

                e = delta.GetEnumerator();
                while (e.MoveNext())
                {
                    if(!hexes.Contains(e.Current))
                    {
                        hexes.Add(e.Current);
                    }
                }
            }

            if (!includeCenter)
            {
                hexes.Remove(centerHexPos);
            }

            return hexes;
        }

        public HexSet Convert(List<HexaPos> hexaPosSet)
        {
            HexSet hexSet = new HexSet(this.mapState);

            List<HexaPos>.Enumerator e = hexaPosSet.GetEnumerator();

            while (e.MoveNext())
            {
                hexSet.hexSet.Add(GetHex(e.Current.X, e.Current.Y));
            }

            return hexSet;
        }

        public Color GetHexColor(int x, int y)
        {
            return mapStateMgr.GetHexColor(x, y);
        }

        public Color GetRGBColor(int x, int y)
        {
            return mapStateMgr.GetRGBColor(x, y);
        }

        public HexaPos GetNext(HexaPos current, Direction direction)
        {
            return GetNext(current.X, current.Y, direction);
        }

        public bool IsAccessible(HexaPos from, HexaPos to)
        {
            int directionLen = directionList.Length;
            for(int l=0;l<directionLen;l++)
            {
                HexaPos nextPos = GetNext(from.X, from.Y, directionList[l]);
                if (to.X==nextPos.X && to.Y==nextPos.Y)
                {
                    return true;
                }
            }
            return false;
        }

        public HexaPos GetNext(int x, int y, Direction direction)
        {
            int nextX = x, nextY = y;
            switch (direction)
            {
                case HexagonalMap.Direction.EAST:
                    if (x < width-1)
                    {
                        nextX = x + 1;
                    }

                    nextY = y;
                    break;
                case HexagonalMap.Direction.WEST:
                    if (x > 0)
                    {
                        nextX = x - 1;
                    }

                    nextY = y;
                    break;
                case HexagonalMap.Direction.NE:
                    if (y > 0)
                    {
                        nextY = y - 1;
                        if (y % 2 == 0)
                        {
                            nextX = x;
                        }
                        else
                        {
                            if ( x < width - 1)
                            {
                                nextX = x + 1;
                            }
                        }
                    }                    
                    break;
                case HexagonalMap.Direction.NW:                    
                    if (y > 0)
                    {
                        nextY = y - 1;
                        if (y % 2 == 0)
                        {
                            if(x > 0)
                            {
                                nextX = x - 1;
                            }
                        }
                        else
                        {
                            nextX = x;
                        }
                    }
                    break;
                     
                case HexagonalMap.Direction.SE:                    
                    if (y < height - 1)
                    {
                        nextY = y + 1;

                        if (y % 2 == 0)
                        {
                            nextX = x;
                        }
                        else
                        {
                            if (x < width - 1)
                            {
                                nextX = x + 1;
                            }
                        }
                    }
                    break;
                case HexagonalMap.Direction.SW:                    
                    if (y < height-1)
                    {
                        nextY = y + 1;
                        if (y % 2 == 0)
                        {
                            if (x > 0)
                            {
                                nextX = x - 1;
                            }
                        }
                        else
                        {
                            nextX = x;
                        }
                    }
                    break;
                case HexagonalMap.Direction.STAY:
                default:

                    nextX = x;
                    nextY = y;

                    break;
            }

            // if next position is an obstacle
            // go back
            if (true == mapState.IsObstacle(GetHex(nextX, nextY)))
            {
                nextX = x;
                nextY = y;
            }

            return new HexaPos(nextX, nextY);
        }

        void InitHexPos()
        {
            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    hexes[i, j].posX = j;
                    hexes[i, j].posY = i;
                }
            }
        }

        /// <param name="width">Board width</param>
        /// <param name="height">Board height</param>
        /// <param name="side">Hexagon side length</param>
        /// <param name="orientation">Orientation of the hexagons</param>
        public HexagonalMap(int width, int height, int side, HexOrientation orientation)
		{
			Initialize(width, height, side, orientation, 0, 0);
            InitHexPos();
		}

		/// <param name="width">Board width</param>
		/// <param name="height">Board height</param>
		/// <param name="side">Hexagon side length</param>
		/// <param name="orientation">Orientation of the hexagons</param>
		/// <param name="xOffset">X coordinate offset</param>
		/// <param name="yOffset">Y coordinate offset</param>
		public HexagonalMap(int width, int height, int side, HexOrientation orientation, int xOffset, int yOffset)
		{
			Initialize(width, height, side, orientation, xOffset, yOffset);
            InitHexPos();
		}

		#region Properties

		public Hex[,] Hexes
		{
			get
			{
				return hexes;
			}
			set
			{
			}
		}

		public float PixelWidth
		{
			get
			{
				return pixelWidth;
			}
			set
			{
			}
		}

		public float PixelHeight
		{
			get
			{
				return pixelHeight;
			}
			set
			{
			}
		}

		public int XOffset
		{
			get
			{
				return xOffset;
			}
			set
			{
			}
		}

		public int YOffset
		{
			get
			{
				return xOffset;
			}
			set
			{
			}
		}

		public int Width
		{
			get
			{
				return width;
			}
			set
			{
			}
		}

		public int Height
		{
			get
			{
				return height;
			}
			set
			{
			}
		}

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

		public HexagonalMapState MapState
		{
			get
			{
				return mapState;
			}
			set
			{
				throw new System.NotImplementedException();
			}
		}

		#endregion 

		/// <summary>
		/// Sets internal fields and creates board
		/// </summary>
		/// <param name="width">Board width</param>
		/// <param name="height">Board height</param>
		/// <param name="side">Hexagon side length</param>
		/// <param name="orientation">Orientation of the hexagons</param>
		/// <param name="xOffset">X coordinate offset</param>
		/// <param name="yOffset">Y coordinate offset</param>
		
		private void Initialize(int width, int height, int side, HexOrientation orientation, int xOffset, int yOffset)
		{
			this.width = width;
			this.height = height;
			this.xOffset = xOffset;
			this.yOffset = yOffset;
			this.side = side;
			this.orientation = orientation;
			hexes = new Hex[height, width]; //opposite of what we'd expect
			this.mapState = new HexagonalMapState(width, height);

			float h = CalcUtil.CalculateH(side); // short side
			float r = CalcUtil.CalculateR(side); // long side

			//
			// Calculate pixel info..remove?
			// because of staggering, need to add an extra r/h
			float hexWidth = 0;
			float hexHeight = 0;
			switch (orientation)
			{
				case HexOrientation.Flat:
					hexWidth = side + h;
					hexHeight = r + r;
					this.pixelWidth = (width * hexWidth) + h;
					this.pixelHeight = (height * hexHeight) + r;
					break;
				case HexOrientation.Pointy:
					hexWidth = r + r;
					hexHeight = side + h;
					this.pixelWidth = (width * hexWidth) + r;
					this.pixelHeight = (height * hexHeight) + h;
					break;
				default:
					break;
			}


			bool inTopRow = false;
			bool inBottomRow = false;
			bool inLeftColumn = false;
			bool inRightColumn = false;
			bool isTopLeft = false;
			bool isTopRight = false;
			bool isBotomLeft = false;
			bool isBottomRight = false;


			// i = y coordinate (rows), j = x coordinate (columns) of the hex tiles 2D plane
			for (int i = 0; i < height; i++)
			{
				for (int j = 0; j < width; j++)
				{
					// Set position booleans
					#region Position Booleans
						if (i == 0)
						{
							inTopRow = true;
						}
						else
						{
							inTopRow = false;
						}

						if (i == height - 1)
						{
							inBottomRow = true;
						}
						else
						{
							inBottomRow = false;
						}

						if (j == 0)
						{
							inLeftColumn = true;
						}
						else
						{
							inLeftColumn = false;
						}

						if (j == width - 1)
						{
							inRightColumn = true;
						}
						else
						{
							inRightColumn = false;
						}

						if (inTopRow && inLeftColumn)
						{
							isTopLeft = true;
						}
						else
						{
							isTopLeft = false;
						}

						if (inTopRow && inRightColumn)
						{
							isTopRight = true;
						}
						else
						{
							isTopRight = false;
						}

						if (inBottomRow && inLeftColumn)
						{
							isBotomLeft = true;
						}
						else
						{
							isBotomLeft = false;
						}

						if (inBottomRow && inRightColumn)
						{
							isBottomRight = true;
						}
						else
						{
							isBottomRight = false;
						}
						#endregion

					//
					// Calculate Hex positions
					//
					if (isTopLeft)
					{
						//First hex
						switch (orientation)
						{ 
							case HexOrientation.Flat:
								hexes[0, 0] = new Hex(0 + h + xOffset, 0 + yOffset, side, orientation);
								break;
							case HexOrientation.Pointy:
								hexes[0, 0] = new Hex(0 + r + xOffset, 0 + yOffset, side, orientation);
								break;
							default:
								break;
						}
					}
					else
					{
						switch (orientation)
						{
							case HexOrientation.Flat:
								if (inLeftColumn)
								{
									// Calculate from hex above
									hexes[i, j] = new Hex(hexes[i - 1, j].Points[(int)FlatVertice.BottomLeft], side, orientation);
								}
								else
								{
									// Calculate from Hex to the left and need to stagger the columns
									if (j % 2 == 0)
									{
										// Calculate from Hex to left's Upper Right Vertice plus h and R offset 
										float x = hexes[i, j - 1].Points[(int)FlatVertice.UpperRight].X;
										float y = hexes[i, j - 1].Points[(int)FlatVertice.UpperRight].Y;
										x += h;
										y -= r;
										hexes[i, j] = new Hex(x, y, side, orientation);
									}
									else
									{
										// Calculate from Hex to left's Middle Right Vertice
										hexes[i, j] = new Hex(hexes[i, j - 1].Points[(int)FlatVertice.MiddleRight], side, orientation);
									}
								}
								break;
							case HexOrientation.Pointy:
								if (inLeftColumn)
								{
									// Calculate from hex above and need to stagger the rows
									if (i % 2 == 0)
									{
										hexes[i, j] = new Hex(hexes[i - 1, j].Points[(int)PointyVertice.BottomLeft], side, orientation);
									}
									else
									{
										hexes[i, j] = new Hex(hexes[i - 1, j].Points[(int)PointyVertice.BottomRight], side, orientation);
									}

								}
								else
								{
									// Calculate from Hex to the left
									float x = hexes[i, j - 1].Points[(int)PointyVertice.UpperRight].X;
									float y = hexes[i, j - 1].Points[(int)PointyVertice.UpperRight].Y;
									x += r;
									y -= h;
									hexes[i, j] = new Hex(x, y, side, orientation);	
								}
								break;
							default:
								break;
						}
					}
				}
			}
            mapStateMgr = new MapStateManager(this);
		}

        public HexagonalMapState GetState()
        {
            return mapState;
        }

		public bool PointInBoardRectangle(System.Drawing.Point point)
		{
			return PointInBoardRectangle(point.X,point.Y);
		}

		public bool PointInBoardRectangle(int x, int y)
		{
			//
			// Quick check to see if X,Y coordinate is even in the bounding rectangle of the board.
			// Can produce a false positive because of the staggerring effect of hexes around the edge
			// of the board, but can be used to rule out an x,y point.
			//
			int topLeftX = 0 + XOffset;
			int topLeftY = 0 + yOffset;
			float bottomRightX = topLeftX + pixelWidth;
			float bottomRightY = topLeftY + PixelHeight;


			if (x > topLeftX && x < bottomRightX && y > topLeftY && y < bottomRightY)
			{
				return true;
			}
			else 
			{
				return false;
			}

		}

		public Hex FindHexMouseClick(System.Drawing.Point point)
		{
			return FindHexMouseClick(point.X,point.Y);
		}

		public Hex FindHexMouseClick(int x, int y)
		{
			Hex target = null;

			if (PointInBoardRectangle(x, y))
			{
				for (int i = 0; i < hexes.GetLength(0); i++)
				{
					for (int j = 0; j < hexes.GetLength(1); j++)
					{
						if (CalcUtil.InsidePolygon(hexes[i, j].Points, 6, new System.Drawing.PointF(x, y)))
						{
							target = hexes[i, j];
							break;
						}
					}

					if (target != null)
					{
						break;
					}
				}

			}
			
			return target;
			
		}

        public MapStateManager GetMapStateMgr()
        {
            return mapStateMgr;
        }

	}
}
