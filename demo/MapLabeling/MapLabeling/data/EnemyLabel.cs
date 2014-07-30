using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;

namespace MapLabeling.data
{
    public class EnemyLabel
    {
        public string name;
        public string id;
        Color pointColor;

        public int size = 13;
        public Point pos;

        public EnemyLabel()
        {
            pointColor = Color.Brown;

        }

        public bool Contains(int x, int y)
        {
            double dist = Math.Sqrt(Math.Pow((double)(x-this.pos.X), 2)+Math.Pow((double)(y-this.pos.Y),2));
            if (dist <= this.size)
            {
                return true;
            }
            return false;
        }

        public void Draw(Graphics graphics)
        {
            SolidBrush mypen = new SolidBrush(pointColor);
            graphics.FillEllipse(mypen, new Rectangle(this.pos.X, this.pos.Y, size, size));
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
