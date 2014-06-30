using System;
using System.Collections.Generic;
using System.Text;
using System.Drawing;
using System.Drawing.Drawing2D;

namespace PathPlanner.Hexagonal
{
	public class HexagonalMapDrawer
	{
		private HexagonalMap map;
		private float mapPixelWidth;
		private float mapPixelHeight;
		private int mapXOffset;
		private int mapYOffset;

        List<Hex> relevantHex;
	
		public HexagonalMapDrawer(HexagonalMap map)
		{
			this.Initialize(map, 0, 0);
		}

		public HexagonalMapDrawer(HexagonalMap map, int xOffset, int yOffset)
		{
			this.Initialize(map, xOffset, yOffset);
		}

		public int BoardXOffset
		{
			get
			{
				return mapXOffset;
			}
			set
			{
				throw new System.NotImplementedException();
			}
		}

		public int BoardYOffset
		{
			get
			{
				return mapYOffset;
			}
			set
			{
				throw new System.NotImplementedException();
			}
		}
		
		private void Initialize(HexagonalMap map, int xOffset, int yOffset)
		{
			this.map = map;
			this.mapXOffset = xOffset;
			this.mapYOffset = yOffset;

            relevantHex = new List<Hex>();
		}

		public void Draw(Graphics graphics)
		{ 
		
			int width =  Convert.ToInt32(System.Math.Ceiling(map.PixelWidth));
			int height = Convert.ToInt32(System.Math.Ceiling(map.PixelHeight));
			// seems to be needed to avoid bottom and right from being chopped off
			width += 1;
			height += 1;

			//
			// Create drawing objects
			//
			Bitmap bitmap = new Bitmap(width, height);
			Graphics bitmapGraphics = Graphics.FromImage(bitmap);
			Pen p = new Pen(Color.Black);
			SolidBrush sb = new SolidBrush(Color.Black);

			//
			// Draw Board background
			//
			sb = new SolidBrush(map.MapState.BackgroundColor);
			bitmapGraphics.FillRectangle(sb, 0, 0, width, height);

			//
			// Draw Hex Background 
			//
			for (int i = 0; i < map.Hexes.GetLength(0); i++)
			{
				for (int j = 0; j < map.Hexes.GetLength(1); j++)
				{
					//bitmapGraphics.DrawPolygon(p, board.Hexes[i, j].Points);
                    bitmapGraphics.FillPolygon(new SolidBrush(map.GetMapStateMgr().GetRGBColor(map.Hexes[i, j].posX, map.Hexes[i, j].posY)), map.Hexes[i, j].Points);
                    //bitmapGraphics.FillPolygon(sb, map.Hexes[i, j].Points);
				}
			}

			//
			// Draw Hex Grid
			//
			p.Color = map.MapState.GridColor;
			p.Width = map.MapState.GridPenWidth;
			for (int i = 0; i < map.Hexes.GetLength(0); i++)
			{
				for (int j = 0; j < map.Hexes.GetLength(1); j++)
				{
					bitmapGraphics.DrawPolygon(p, map.Hexes[i, j].Points);
				}
			}

			//
			// Draw Active Hex, if present
			//
			if (map.MapState.ActiveHex != null)
			{
				p.Color = map.MapState.ActiveHexBorderColor;
				p.Width = map.MapState.ActiveHexBorderWidth;
				bitmapGraphics.DrawPolygon(p, map.MapState.ActiveHex.Points);
                //bitmapGraphics.FillPolygon(new SolidBrush(p.Color), map.MapState.ActiveHex.Points);
   
                /*
                List<HexaPos> tempList = map.GetHexes(map.MapState.ActiveHex.posX, map.MapState.ActiveHex.posY, 1);
                List<HexaPos>.Enumerator e = tempList.GetEnumerator();
                while (e.MoveNext())
                {
                    Hex tempHex = map.GetHex(e.Current.X, e.Current.Y);

                    p.Color = Color.Blue;
                    bitmapGraphics.DrawPolygon(p, tempHex.Points);
                }
                 */ 
			}

            if (map.MapState.ActiveHex2 != null)
            {
                p.Color = map.MapState.ActiveHexBorderColor2;
                p.Width = map.MapState.ActiveHexBorderWidth;
                bitmapGraphics.DrawPolygon(p, map.MapState.ActiveHex2.Points);
                //bitmapGraphics.FillPolygon(new SolidBrush(p.Color), map.MapState.ActiveHex.Points);

                /*
                List<HexaPos> tempList = map.GetHexes(map.MapState.ActiveHex.posX, map.MapState.ActiveHex.posY, 1);
                List<HexaPos>.Enumerator e = tempList.GetEnumerator();
                while (e.MoveNext())
                {
                    Hex tempHex = map.GetHex(e.Current.X, e.Current.Y);

                    p.Color = Color.Blue;
                    bitmapGraphics.DrawPolygon(p, tempHex.Points);
                }
                 */
            }

            if (map.MapState.ActiveHexSet != null)
            {
                HexSet activeHexSet = map.MapState.ActiveHexSet;
                List<Hex>.Enumerator e =  activeHexSet.hexSet.GetEnumerator();
                while (e.MoveNext())
                {
                    p.Color = Color.Yellow;
                    bitmapGraphics.FillPolygon(new SolidBrush(p.Color), e.Current.Points);

                }
            }

            List<Hex>.Enumerator eO = map.MapState.obstacles.hexSet.GetEnumerator();
            while (eO.MoveNext())
            {
                bitmapGraphics.FillPolygon(new SolidBrush(Color.Chocolate), eO.Current.Points);
            }

            int setNum = map.MapState.GetHexSetNum();
            if (setNum > 0)
            {
                for (int i = 0; i < setNum; i++)
                {
                    HexSet set = map.MapState.GetHexSet(i);

                    int setSize = set.hexSet.Count;
                    for (int j = 0; j < setSize; j++)
                    {
                        Hex tempHex = set.hexSet[j];
                        p.Color = set.borderColor;
                        //bitmapGraphics.DrawPolygon(p, tempHex.Points);
                        bitmapGraphics.FillPolygon(new SolidBrush(p.Color), tempHex.Points);


                        Pen lightPen = new Pen(Color.LightYellow);
                        lightPen.Width = map.MapState.GridPenWidth;
                        bitmapGraphics.DrawPolygon(lightPen, tempHex.Points);
                    }
                }
            }

			//
			// Draw internal bitmap to screen
			//
			graphics.DrawImage(bitmap, new Point(this.mapXOffset , this.mapYOffset));
			
			//
			// Release objects
			//
			bitmapGraphics.Dispose();
			bitmap.Dispose();
		}

        public void DrawEnv(string filename, double[,] entropy, HexaPath path, Color pathColor, HexaPath refPath)
        {
            int width = Convert.ToInt32(System.Math.Ceiling(map.PixelWidth));
            int height = Convert.ToInt32(System.Math.Ceiling(map.PixelHeight));
            width += 1;
            height += 1;
            Bitmap bitmap = new Bitmap(width, height);
            Graphics bitmapGraphics = Graphics.FromImage(bitmap);
            Pen p = new Pen(Color.Black);
            SolidBrush sb = new SolidBrush(Color.Black);

            sb = new SolidBrush(map.MapState.BackgroundColor);
            bitmapGraphics.FillRectangle(sb, 0, 0, width, height);
            for (int i = 0; i < map.Hexes.GetLength(0); i++)
            {
                for (int j = 0; j < map.Hexes.GetLength(1); j++)
                {
                    //bitmapGraphics.FillPolygon(new SolidBrush(map.GetMapStateMgr().GetColor(entropy[map.Hexes[i, j].posX, map.Hexes[i, j].posY])), map.Hexes[i, j].Points);
                    bitmapGraphics.FillPolygon(sb, map.Hexes[i, j].Points);
                }
            }
            p.Color = map.MapState.GridColor;
            p.Width = map.MapState.GridPenWidth;
            for (int i = 0; i < map.Hexes.GetLength(0); i++)
            {
                for (int j = 0; j < map.Hexes.GetLength(1); j++)
                {
                    bitmapGraphics.DrawPolygon(p, map.Hexes[i, j].Points);
                }
            }

            List<Hex>.Enumerator eO = map.MapState.obstacles.hexSet.GetEnumerator();
            while (eO.MoveNext())
            {
                bitmapGraphics.FillPolygon(new SolidBrush(Color.Chocolate), eO.Current.Points);
            }

            if (refPath != null)
            {
                int pathLen = refPath.Length;

                if (pathLen > 0)
                {
                    for (int i = 0; i < pathLen; i++)
                    {
                        Hex tempHex = map.GetHex(refPath[i].X, refPath[i].Y);
                        p.Color = Color.Blue;
                        bitmapGraphics.DrawPolygon(p, tempHex.Points);
                    }
                }
            }

            if (path != null)
            {
                int pathLen = path.Length;

                if (pathLen > 0)
                {
                    for (int i = 0; i < pathLen; i++)
                    {
                        Hex tempHex = map.GetHex(path[i].X, path[i].Y);
                        p.Color = pathColor;
                        bitmapGraphics.DrawPolygon(p, tempHex.Points);
                    }
                }
            }

            bitmap.Save(filename);
        }
	}
}
