using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Xml;
using System.IO;

namespace PathPlanner.Data
{
    public class OutdoorLabelManager
    {
        public List<OutdoorLabel> outdoors;
        int counter = 0;
        string prefix = "Outdoor-";
        public OutdoorLabel activeOutdoor;
        public MapInfoManager parent;

        public OutdoorLabelManager(MapInfoManager parent)
        {
            this.parent = parent;
            outdoors = new List<OutdoorLabel>();
        }

        public bool FindActiveOutdoor(int x, int y)
        {
            for (int i = 0; i < outdoors.Count; i++)
            {
                if (true == outdoors[i].Contains(x, y))
                {
                    activeOutdoor = outdoors[i];
                    return true;
                }
            }
            return false;
        }

        public void ResetActiveOutdoor()
        {
            activeOutdoor = null;
        }

        public void DeleteActiveOutdoor()
        {
            OutdoorLabel temp = activeOutdoor;
            ResetActiveOutdoor();
            outdoors.Remove(temp);
        }

        public OutdoorLabel CreateLabel()
        {
            OutdoorLabel label = new OutdoorLabel();
            label.id = this.prefix + counter.ToString();

            return label;
        }

        public bool IsLabelInList(string labelId)
        {
            List<OutdoorLabel>.Enumerator e = outdoors.GetEnumerator();
            while (e.MoveNext())
            {
                if (e.Current.id == labelId)
                {
                    return true;
                }
            }
            return false;
        }

        public void AddLabel(OutdoorLabel label)
        {
            if (false == IsLabelInList(label.id))
            {
                outdoors.Add(label);
            }

            counter++;
        }

        public void Draw(Graphics graphics)
        {
            foreach (OutdoorLabel p in this.outdoors)
            {
                p.Draw(graphics);
            }
            if (activeOutdoor != null)
            {
                Pen solidPen = new Pen(Color.Red);
                solidPen.Width = 4;
                graphics.DrawPolygon(solidPen, activeOutdoor.vertices.ToArray());

                string info = "ID: " + activeOutdoor.id;
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

                #region Outdoors
                
                foreach (OutdoorLabel label in this.outdoors)
                {
                    xtw.WriteStartElement("Outdoor");
                    xtw.WriteAttributeString("ID", label.id);
                    xtw.WriteAttributeString("Name", label.name);
                    xtw.WriteAttributeString("Type", label.WrtieType(label.type));

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

                    this.outdoors.Clear();

                    do
                    {
                        xtr.Read();
                        if (xtr.NodeType == XmlNodeType.None)
                        {
                            return;
                        }
                    }
                    while (!(xtr.NodeType == XmlNodeType.Element && xtr.Name == "Outdoor"));


                    while ((xtr.NodeType == XmlNodeType.Element && xtr.Name == "Outdoor"))
                    {

                        OutdoorLabel label = new OutdoorLabel();

                        label.id = xtr.GetAttribute("ID");
                        label.name = xtr.GetAttribute("Name");
                        label.type = label.ReadType(xtr.GetAttribute("Type"));

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

                        this.outdoors.Add(label);

                        while (!(xtr.NodeType == XmlNodeType.EndElement && xtr.Name == "Outdoor"))
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
