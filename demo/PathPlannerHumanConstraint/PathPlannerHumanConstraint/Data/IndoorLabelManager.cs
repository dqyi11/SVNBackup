using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Xml;
using System.IO;

namespace PathPlanner.Data
{
    public class IndoorLabelManager
    {
        public List<IndoorLabel> indoors;
        int counter = 0;
        string prefix = "Indoor-";
        public IndoorLabel activeIndoor;
        public MapInfoManager parent;

        public IndoorLabelManager(MapInfoManager parent)
        {
            this.parent = parent;
            indoors = new List<IndoorLabel>();
        }

        public bool FindActiveIndoor(int x, int y)
        {
            for (int i = 0; i < indoors.Count; i++)
            {
                if (true == indoors[i].Contains(x, y))
                {
                    activeIndoor = indoors[i];
                    return true;
                }
            }
            return false;
        }

        public void ResetActiveIndoor()
        {
            activeIndoor = null;
        }

        public void DeleteActiveIndoor()
        {
            IndoorLabel temp = activeIndoor;
            ResetActiveIndoor();
            indoors.Remove(temp);
        }

        public IndoorLabel CreateLabel()
        {
            IndoorLabel label = new IndoorLabel();
            label.id = this.prefix + counter.ToString();

            return label;
        }

        public bool IsLabelInList(string labelId)
        {
            List<IndoorLabel>.Enumerator e = indoors.GetEnumerator();
            while (e.MoveNext())
            {
                if (e.Current.id == labelId)
                {
                    return true;
                }
            }
            return false;
        }

        public void AddLabel(IndoorLabel label)
        {
            if (false == IsLabelInList(label.id))
            {
                indoors.Add(label);
            }

            counter++;
        }

        public void Draw(Graphics graphics)
        {
            foreach (IndoorLabel p in this.indoors)
            {
                p.Draw(graphics);
            }
            if (activeIndoor != null)
            {
                Pen solidPen = new Pen(Color.Red);
                solidPen.Width = 4;
                graphics.DrawPolygon(solidPen, activeIndoor.vertices.ToArray());

                string info = "ID: " + activeIndoor.id;
                Font str_font = new Font("Verdana", 12);
                System.Drawing.SizeF str_size = graphics.MeasureString(info, str_font);

                graphics.FillRectangle(new SolidBrush(Color.LightCoral), new Rectangle(this.parent.activeX + 20, this.parent.activeY, (int)str_size.Width + 1, (int)str_size.Height));
                graphics.DrawString(info, str_font, new SolidBrush(Color.Black), new Rectangle(this.parent.activeX + 20, this.parent.activeY, (int)str_size.Width + 1, (int)str_size.Height)); 
            }
        }

        public string DumpToString()
        {
            using (StringWriter sw = new StringWriter())
            {
                XmlTextWriter xtw = new XmlTextWriter(sw);

                #region Indoors

                foreach (IndoorLabel label in this.indoors)
                {
                    xtw.WriteStartElement("Indoor");
                    xtw.WriteAttributeString("ID", label.id);
                    xtw.WriteAttributeString("Name", label.name);

                    xtw.WriteStartElement("Vertices");

                    foreach (Point p in label.vertices)
                    {
                        xtw.WriteStartElement("Vertex");
                        xtw.WriteAttributeString("Pos", p.ToString());
                        xtw.WriteEndElement();
                    }

                    xtw.WriteEndElement();
                    
                    xtw.WriteEndElement();

                }
            
                #endregion

                return sw.ToString();
            }
        }

        public void LoadFromString(string input)
        {
            try
            {
                using (StringReader sr = new StringReader(input))
                {
                    XmlTextReader xtr = new XmlTextReader(sr);

                    this.indoors.Clear();

                    do
                    {
                        xtr.Read();
                        if (xtr.NodeType == XmlNodeType.None)
                        {
                            return;
                        }
                    }
                    while (!(xtr.NodeType == XmlNodeType.Element && xtr.Name == "Indoor"));


                    while ((xtr.NodeType == XmlNodeType.Element && xtr.Name == "Indoor"))
                    {
                        IndoorLabel label = new IndoorLabel();

                        label.id = xtr.GetAttribute("ID");
                        label.name = xtr.GetAttribute("Name");

                        do
                        {
                            xtr.Read();
                        } while (!(xtr.NodeType == XmlNodeType.Element && xtr.Name == "Vertices"));

                        {
                            while (xtr.Read() && !(xtr.Name == "Vertices" && xtr.NodeType == XmlNodeType.EndElement))
                            {
                                if (xtr.Name == "Vertex")
                                {
                                    label.vertices.Add(label.ReadPoint(xtr.GetAttribute("Pos")));
                                }
                            }
                        }

                        label.Complete();

                        this.indoors.Add(label);

                        while (!(xtr.NodeType == XmlNodeType.EndElement && xtr.Name == "Indoor"))
                        {
                            xtr.Read();
                        }

                        xtr.Read();

                    }
                }

            }
            catch (System.Exception ex)
            {
                throw ex;
            }
        }

    }
}
