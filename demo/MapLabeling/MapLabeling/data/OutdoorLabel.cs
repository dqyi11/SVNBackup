using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Drawing.Drawing2D;

namespace MapLabeling.data
{
    public class OutdoorLabel
    {
        public enum OUTDOOR_TYPE
        {
            UNKNOWN = 0,
            MARKET = 1,
            RISKY = 2,
        }

        public OUTDOOR_TYPE[] types = 
        {
            OUTDOOR_TYPE.UNKNOWN,
            OUTDOOR_TYPE.MARKET,
            OUTDOOR_TYPE.RISKY,
        };

        public string[] TYPE_SYMBOL =
        {
            "U",
            "M",
            "R",
        };

        public string[] TYPE_STR = 
        {
            "Unknown",
            "Market",
            "Risky",
        };

        public string name;
        public string id;
        Color lineColor;
        bool completed;

        public List<Point> vertices;
        public Point center;

        public OUTDOOR_TYPE type;

        public OutdoorLabel()
        {
            lineColor = Color.YellowGreen;
            completed = false;
            vertices = new List<Point>();
            type = OUTDOOR_TYPE.UNKNOWN;
        }

        public bool IsCompleted()
        {
            return completed;
        }

        public void Complete()
        {
            completed = true;
            int num = vertices.Count;
            int totalX = 0, totalY = 0;
            foreach (Point v in this.vertices)
            {
                totalX += v.X;
                totalY += v.Y;
            }
            center.X = totalX / num;
            center.Y = totalY / num;
        }

        public void Clear()
        {
            vertices.Clear();
        }

        public bool Contains(int x, int y)
        {
            int j = vertices.Count - 1;
            bool oddNodes = false;
            for (int i = 0; i < vertices.Count; i++)
            {
                if (vertices[i].Y < y && vertices[j].Y >= y ||
                    vertices[j].Y < y && vertices[i].Y >= y)
                {
                    if (vertices[i].X + (y - vertices[i].Y) / (vertices[j].Y - vertices[i].Y) * (vertices[j].X - vertices[i].X) < x)
                    {
                        oddNodes = !oddNodes;
                    }
                }
                j = i;
            }

            return oddNodes;
        }


        public void AddVertice(Point vertice)
        {
            vertices.Add(vertice);
        }

        public void Draw(Graphics graphics)
        {
            if (this.completed == false)
            {
                Pen pen = new Pen(lineColor);
                // draw dot line
                int verticeNum = vertices.Count;
                for (int i = 0; i < verticeNum - 1; i++)
                {
                    pen.DashStyle = System.Drawing.Drawing2D.DashStyle.Dash;
                    graphics.DrawLine(pen, vertices[i], vertices[i + 1]);
                }

                foreach (Point p in vertices)
                {
                    graphics.DrawEllipse(pen, p.X-2, p.Y-2, 4, 4);
                }
            }
            else
            {
                // draw solid line
                if (this.type == OUTDOOR_TYPE.UNKNOWN)
                {
                    Pen solidPen = new Pen(Color.FromArgb(80, lineColor));
                    solidPen.Width = 4;
                    //graphics.DrawPolygon(solidPen, vertices.ToArray());
                    graphics.FillPolygon(new SolidBrush(Color.FromArgb(80, lineColor)), vertices.ToArray());
                }
                else if (this.type == OUTDOOR_TYPE.RISKY)
                {
                    Pen dashPen = new Pen(Color.FromArgb(80, lineColor));
                    dashPen.DashStyle = DashStyle.DashDot;
                    dashPen.Width = 4;
                    //graphics.DrawPolygon(dashPen, vertices.ToArray());
                    graphics.FillPolygon(new SolidBrush(Color.FromArgb(80, lineColor)), vertices.ToArray());
                }
                else if(this.type == OUTDOOR_TYPE.MARKET)
                {
                    graphics.FillPolygon(new SolidBrush(Color.FromArgb(80, lineColor)), vertices.ToArray());
                }
                //graphics.DrawPolygon(new Pen(lineColor), vertices.ToArray());
                //graphics.DrawString(this.name, SystemFonts.DefaultFont, new SolidBrush(lineColor), center);

                if (this.name != null && this.name.Length > 0)
                {
              
                    //graphics.FillRectangle(new SolidBrush(Color.FromArgb(80, Color.White)), new Rectangle(center.X, center.Y, 100, 30));
                    //graphics.DrawString(this.name, new Font("Verdana", 20), new SolidBrush(lineColor), new Rectangle(center.X, center.Y, 100, 30));
                    Font str_font = new Font("Verdana", 20);
                    System.Drawing.SizeF str_size = graphics.MeasureString(this.name, str_font);
                    //graphics.DrawString(this.name, str_font, new SolidBrush(lineColor), new Rectangle(this.center.X + 20, this.center.Y, (int)str_size.Width+1, (int)str_size.Height)); 
                }
            }
        }

        public OUTDOOR_TYPE ReadType(string typeStr)
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

        public string WrtieType(OUTDOOR_TYPE type)
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
    }
}
