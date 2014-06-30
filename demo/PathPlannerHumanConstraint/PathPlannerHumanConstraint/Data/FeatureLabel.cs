using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;

namespace PathPlanner.Data
{
    public class FeatureLabel
    {
        public enum FEATURE_TYPE
        {
            DOOR = 1,
            WINDOW = 2,
            UNKNOWN,
        }

        public FEATURE_TYPE[] types =
        {
            FEATURE_TYPE.DOOR,
            FEATURE_TYPE.WINDOW,
            FEATURE_TYPE.UNKNOWN,
        };

        public string[] TYPE_SYMBOL =
        {
            "D",
            "W",
            " ",
        };

        public string[] TYPE_STR = 
        {
            "Door",
            "Window",
            "Unknown",
        };

        public string name;
        public string id;
        public FEATURE_TYPE type;
        Color pointColor;

        public int size = 13;

        public Point pos;

        public FeatureLabel()
        {
            pointColor = Color.DarkOrange;
            type = FEATURE_TYPE.DOOR;
        }

        public bool Contains(int x, int y)
        {
            List<Point> points = new List<Point>();
            points.Add(new Point(this.pos.X, this.pos.Y));
            points.Add(new Point(this.pos.X + this.size, this.pos.Y));
            points.Add(new Point(this.pos.X + this.size, this.pos.Y + this.size));
            points.Add(new Point(this.pos.X, this.pos.Y + this.size));

            int j = points.Count - 1;
            bool oddNodes = false;
            for (int i = 0; i < points.Count; i++)
            {
                if (points[i].Y < y && points[j].Y >= y ||
                    points[j].Y < y && points[i].Y >= y)
                {
                    if (points[i].X + (y - points[i].Y) / (points[j].Y - points[i].Y) * (points[j].X - points[i].X) < x)
                    {
                        oddNodes = !oddNodes;
                    }
                }
                j = i;
            }

            return oddNodes;
        }

        public void Draw(Graphics graphics)
        {
            SolidBrush mypen = new SolidBrush(pointColor);
            graphics.FillRectangle(mypen, new Rectangle(this.pos.X, this.pos.Y, size, size));
            //graphics.FillRectangle(new SolidBrush(Color.FromArgb(80, pointColor)), new Rectangle(this.pos.X, this.pos.Y, 13, 13));
            //graphics.DrawString(TYPE_SYMBOL[(int)this.type], SystemFonts.DefaultFont, new SolidBrush(pointColor), new Rectangle(this.pos.X, this.pos.Y, 13, 13));
            if (this.name != null && this.name.Length > 0)
            {
                //graphics.FillRectangle(new SolidBrush(Color.FromArgb(80, Color.White)),new Rectangle(this.pos.X + 20, this.pos.Y, 100, 15));
                Font str_font = new Font("Verdana", 20);
                System.Drawing.SizeF str_size = graphics.MeasureString(this.name, str_font);
                //graphics.DrawString(this.name, str_font, new SolidBrush(pointColor), new Rectangle(this.pos.X + 20, this.pos.Y, (int)str_size.Width+1, (int)str_size.Height)); 
            }
        }

        public Point ReadPoint(string pointStr)
        {
            Point p = new Point();
            string temp = pointStr.Replace("{X=", "");
            temp = temp.Replace("Y=", "");
            temp = temp.Replace("}", "");

            int mid = temp.IndexOf(",");
            string xStr = temp.Substring(0, mid);
            string yStr = temp.Substring(mid + 1);
            p.X = Int32.Parse(xStr);
            p.Y = Int32.Parse(yStr);

            return p;
        }

        public FEATURE_TYPE ReadType(string typeStr)
        {
            for (int i = 0; i < TYPE_STR.Length; i++)
            {
                if (typeStr == TYPE_STR[i])
                {
                    return types[i];
                }
            }
            return types[types.Length - 1];
        }

        public string WrtieType(FEATURE_TYPE type)
        {
            for (int i = 0; i < types.Length; i++)
            {
                if (type == types[i])
                {
                    return TYPE_STR[i];
                }
            }
            return TYPE_STR[TYPE_STR.Length - 1];
        }
        
    }
}
