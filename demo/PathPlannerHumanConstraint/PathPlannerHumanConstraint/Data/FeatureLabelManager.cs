using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Xml;
using System.IO;

namespace PathPlanner.Data
{
    public class FeatureLabelManager
    {
        public List<FeatureLabel> features;
        int counter = 0;
        string prefix = "Feature-";
        public FeatureLabel activeFeature;
        public MapInfoManager parent;

        public FeatureLabelManager(MapInfoManager parent)
        {
            this.parent = parent;
            features = new List<FeatureLabel>();
        }

        public bool FindActiveFeature(int x, int y)
        {
            for (int i = 0; i < features.Count; i++)
            {
                if (true == features[i].Contains(x, y))
                {
                    activeFeature = features[i];
                    return true;
                }
            }
            return false;
        }

        public void ResetActiveFeature()
        {
            activeFeature = null;
        }

        public void DeleteActiveFeature()
        {
            FeatureLabel temp = activeFeature;
            ResetActiveFeature();
            features.Remove(temp);
        }

        public FeatureLabel CreateLabel()
        {
            FeatureLabel label = new FeatureLabel();
            label.id = this.prefix + counter.ToString();

            return label;
        }

        public bool IsLabelInList(string labelId)
        {
            List<FeatureLabel>.Enumerator e = features.GetEnumerator();
            while (e.MoveNext())
            {
                if (e.Current.id == labelId)
                {
                    return true;
                }
            }
            return false;
        }

        public void AddLabel(FeatureLabel label)
        {
            if(false==IsLabelInList(label.id))
            {
                features.Add(label);
            }

            counter++;
        }

        public void Draw(Graphics graphics)
        {
            foreach (FeatureLabel p in this.features)
            {
                p.Draw(graphics);
            }
            if (activeFeature != null)
            {
                Pen solidPen = new Pen(Color.Red);
                solidPen.Width = 4;
                graphics.DrawRectangle(solidPen, new Rectangle(activeFeature.pos.X, activeFeature.pos.Y, activeFeature.size, activeFeature.size));

                string info = "ID: " + activeFeature.id;
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

                #region Features
                
                foreach (FeatureLabel label in this.features)
                {
                    xtw.WriteStartElement("Feature");
                    xtw.WriteAttributeString("ID", label.id);
                    xtw.WriteAttributeString("Name", label.name);
                    xtw.WriteAttributeString("Type", label.WrtieType(label.type));
                    xtw.WriteAttributeString("Pos", label.pos.ToString());
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

                    this.features.Clear();

                    do
                    {
                        xtr.Read();

                        if (xtr.NodeType == XmlNodeType.None)
                        {
                            return;
                        }
                    }
                    while (!(xtr.NodeType == XmlNodeType.Element && xtr.Name == "Feature"));


                    while ((xtr.NodeType == XmlNodeType.Element && xtr.Name == "Feature"))
                    {
                        FeatureLabel label = new FeatureLabel();

                        label.id = xtr.GetAttribute("ID");
                        label.name = xtr.GetAttribute("Name");
                        label.type = label.ReadType(xtr.GetAttribute("Type"));
                        label.pos = label.ReadPoint(xtr.GetAttribute("Pos"));

                        this.features.Add(label);

                        while (!(xtr.NodeType == XmlNodeType.EndElement && xtr.Name == "Feature")
                            && !(xtr.NodeType == XmlNodeType.EndElement && xtr.Name == "features"))
                        {
                            xtr.Read();
                        }

                        if (xtr.NodeType != XmlNodeType.None)
                        {
                            xtr.Read();
                        }

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
