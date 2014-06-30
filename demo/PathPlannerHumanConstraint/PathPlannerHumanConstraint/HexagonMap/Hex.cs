using System;
using System.Collections.Generic;
using System.Text;
using System.Drawing;

namespace PathPlanner.Hexagonal
{
	public class Hex
	{
		private System.Drawing.PointF[] points;
		private float side;
		private float h;
		private float r;
		private HexOrientation orientation;
		private float x;
		private float y;
		private HexState hexState;

        public System.Drawing.PointF centroid;
        public List<System.Drawing.PointF> inPointSet;

        public int posX;
        public int posY;
	
		/// <param name="side">length of one side of the hexagon</param>
		public Hex(int x, int y, int side, HexOrientation orientation)
		{
			Initialize(CalcUtil.ConvertToFloat(x), CalcUtil.ConvertToFloat(y), CalcUtil.ConvertToFloat(side),orientation);
		}

		public Hex(float x, float y, float side, HexOrientation orientation)
		{
			Initialize(x, y, side, orientation);
		}

		public Hex(PointF point, float side, HexOrientation orientation)
		{
			Initialize(point.X, point.Y, side, orientation);
		}

        public override bool Equals(object obj)
        {
            if (this.x == ((Hex)obj).x 
                && this.y == ((Hex)obj).y)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        public bool Equals(Hex hex)
        {
            if (this.x == hex.x && this.y == hex.y)
            {
                return true;
            }

            return false;
        }

		public Hex()
		{ }

		/// <summary>
		/// Sets internal fields and calls CalculateVertices()
		/// </summary>
		private void Initialize(float x, float y, float side, HexOrientation orientation)
		{
			this.x = x;
			this.y = y;
			this.side = side;
			this.orientation = orientation;
			this.hexState = new HexState();
			CalculateVertices();
		}

		/// <summary>
		/// Calculates the vertices of the hex based on orientation. Assumes that points[0] contains a value.
		/// </summary>
		private void CalculateVertices()
		{
			//  
			//  h = short length (outside)
			//  r = long length (outside)
			//  side = length of a side of the hexagon, all 6 are equal length
			//
			//  h = sin (30 degrees) x side
			//  r = cos (30 degrees) x side
			//
			//		 h
			//	     ---
			//   ----     |r
			//  /    \    |          
			// /      \   |
			// \      /
			//  \____/
			//
			// Flat orientation (scale is off)
			//
	        //     /\
			//    /  \
			//   /    \
			//   |    |
			//   |    |
			//   |    |
			//   \    /
			//    \  /
			//     \/
			// Pointy orientation (scale is off)
         
			h = CalcUtil.CalculateH(side);
			r = CalcUtil.CalculateR(side);

            int x_low, x_high, y_low, y_high = 0;

			switch (orientation)
			{ 
				case HexOrientation.Flat:
					// x,y coordinates are top left point
					points = new System.Drawing.PointF[6];
					points[0] = new PointF(x, y);
					points[1] = new PointF(x + side, y);
					points[2] = new PointF(x + side + h, y + r);
					points[3] = new PointF(x + side, y + r + r);
					points[4] = new PointF(x, y + r + r);
					points[5] = new PointF(x - h, y + r );

                    x_low = (int)System.Math.Ceiling(x - h);
                    x_high = (int)System.Math.Floor(x + side + h);
                    y_low = (int)System.Math.Ceiling(y);
                    y_high = (int)System.Math.Floor(y + r + r);
					break;
				case HexOrientation.Pointy:
					//x,y coordinates are top center point
					points = new System.Drawing.PointF[6];
					points[0] = new PointF(x, y);
					points[1] = new PointF(x + r, y + h);
					points[2] = new PointF(x + r, y + side + h);
					points[3] = new PointF(x, y + side + h + h);
					points[4] = new PointF(x - r, y + side + h);
					points[5] = new PointF(x - r, y + h);

                    x_low = (int)System.Math.Ceiling(x - r);
                    x_high = (int)System.Math.Floor(x + r);
                    y_low = (int)System.Math.Ceiling(y);
                    y_high = (int)System.Math.Floor(y + side + h);
					break;
				default:
					throw new Exception("No HexOrientation defined for Hex object.");
			
			}

            float cX = 0;
            float cY = 0;
            for (int i = 0; i < 6; i++)
            {
                cX += points[i].X;
                cY += points[i].Y;
            }
            centroid = new PointF(cX/6, cY/6);

            inPointSet = new List<System.Drawing.PointF> ();
            for (int iX = x_low; iX <= x_high; iX++)
            {
                for (int iY = y_low; iY <= y_high; iY++)
                {
                    PointF p = new PointF(iX, iY);
                    if (CalcUtil.InsidePolygon(points, 6, p))
                    {
                        inPointSet.Add(p);
                    }
                }
            }

		}

        public HexOrientation Orientation
		{
			get
			{
				return orientation;
			}
			set
			{
				orientation = value;
			}
		}

		public System.Drawing.PointF[] Points
		{
			get
			{
				return points;
			}
			set
			{
			}
		}

		public float Side
		{
			get
			{
				return side;
			}
			set
			{
			}
		}

		public float H
		{
			get
			{
				return h;
			}
			set
			{
			}
		}

		public float R
		{
			get
			{
				return r;
			}
			set
			{
			}
		}

		public HexState HexState
		{
			get
			{
				return hexState;
			}
			set
			{
				throw new System.NotImplementedException();
			}
		}

	}
}
