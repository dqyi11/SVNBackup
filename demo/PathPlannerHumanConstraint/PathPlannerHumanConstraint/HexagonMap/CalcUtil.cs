using System;
using System.Collections.Generic;
using System.Text;
using System.Drawing;

namespace PathPlanner.Hexagonal
{
	class CalcUtil
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

		public static double DegreesToRadians(double degrees)
		{
			//http://en.wikipedia.org/wiki/Radians
			return degrees * System.Math.PI / 180;
		}

		public static double RadiansToDegrees(double radians)
		{
			return radians * 180 / System.Math.PI;
		}

		public static float ConvertToFloat(double d)
		{
			return (float)d;
		}

		public static float ConvertToFloat(int i)
		{
			return (float)i;
		}

		/// <summary>
		/// Outside triangle side (short)
		/// </summary>
		public static float CalculateH(float side)
		{
			return ConvertToFloat(System.Math.Sin(DegreesToRadians(30)) * side);
		}

		/// <summary>
		/// Outside triangle side (long)
		/// </summary>
		public static float CalculateR(float side)
		{
			return ConvertToFloat(System.Math.Cos(DegreesToRadians(30)) * side);
		}


		public static bool InsidePolygon(PointF[] polygon, int N, PointF p)
		{
			//http://astronomy.swin.edu.au/~pbourke/geometry/insidepoly/
			//
			// Slick algorithm that checks if a point is inside a polygon.  Checks how may time a line
			// origination from point will cross each side.  An odd result means inside polygon.
			//
			int counter = 0;
			int i;
			double xinters;
			PointF p1,p2;
			
			p1 = polygon[0];
			for (i=1;i<=N;i++) 
			{
				p2 = polygon[i % N];
				if (p.Y > System.Math.Min(p1.Y,p2.Y)) 
				{
					if (p.Y <= System.Math.Max(p1.Y,p2.Y)) 
					{
						if (p.X <= System.Math.Max(p1.X,p2.X)) 
						{
							if (p1.Y != p2.Y) 
							{
								xinters = (p.Y-p1.Y)*(p2.X-p1.X)/(p2.Y-p1.Y)+p1.X;
								if (p1.X == p2.X || p.X <= xinters)
									counter++;
							}
						}
					}
				}	
				p1 = p2;
			}

			if (counter % 2 == 0)
				return false;
			else
				return true;
		}

		
	}
}