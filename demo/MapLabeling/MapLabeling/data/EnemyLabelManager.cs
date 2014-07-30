using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Xml;
using System.IO;

namespace MapLabeling.data
{
    public class EnemyLabelManager
    {
        public List<EnemyLabel> enemies;
        public MapInfoManager parent;
        public EnemyLabel activeEnemy;
        int counter = 0;
        string prefix = "Enemy-";

        public EnemyLabelManager(MapInfoManager parent)
        {
            this.parent = parent;
            enemies = new List<EnemyLabel>();
        }

        public bool FindActiveEnemy(int x, int y)
        {
            for (int i = 0; i < enemies.Count; i++)
            {
                if (true == enemies[i].Contains(x, y))
                {
                    activeEnemy = enemies[i];
                    return true;
                }
            }
            return false;
        }

        public void ResetActiveEnemy()
        {
            activeEnemy = null;
        }

        public void DeleteActiveFeature()
        {
            EnemyLabel temp = activeEnemy;
            ResetActiveEnemy();
            enemies.Remove(temp);
        }

        public EnemyLabel CreateLabel()
        {
            EnemyLabel label = new EnemyLabel();
            label.id = this.prefix + counter.ToString();

            return label;
        }

        public bool IsLabelInList(string labelId)
        {
            List<EnemyLabel>.Enumerator e = enemies.GetEnumerator();
            while (e.MoveNext())
            {
                if (e.Current.id == labelId)
                {
                    return true;
                }
            }
            return false;
        }

        public void AddLabel(EnemyLabel label)
        {
            if (false == IsLabelInList(label.id))
            {
                enemies.Add(label);
            }

            counter++;
        }

        public void Draw(Graphics graphics)
        {
            foreach (EnemyLabel p in this.enemies)
            {
                p.Draw(graphics);
            }
            if (activeEnemy != null)
            {
                Pen solidPen = new Pen(Color.Red);
                solidPen.Width = 4;
                graphics.DrawEllipse(solidPen, new Rectangle(activeEnemy.pos.X, activeEnemy.pos.Y, activeEnemy.size, activeEnemy.size));

                string info = "ID: " + activeEnemy.id;
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

                foreach (EnemyLabel label in this.enemies)
                {
                    xtw.WriteStartElement("Enemy");
                    xtw.WriteAttributeString("ID", label.id);
                    xtw.WriteAttributeString("Name", label.name);
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

                    this.enemies.Clear();

                    do
                    {
                        xtr.Read();

                        if (xtr.NodeType == XmlNodeType.None)
                        {
                            return;
                        }
                    }
                    while (!(xtr.NodeType == XmlNodeType.Element && xtr.Name == "Enemy"));


                    while ((xtr.NodeType == XmlNodeType.Element && xtr.Name == "Enemy"))
                    {
                        EnemyLabel label = new EnemyLabel();

                        label.id = xtr.GetAttribute("ID");
                        label.name = xtr.GetAttribute("Name");
                        label.pos = label.ReadPoint(xtr.GetAttribute("Pos"));

                        this.enemies.Add(label);

                        while (!(xtr.NodeType == XmlNodeType.EndElement && xtr.Name == "Enemy")
                            && !(xtr.NodeType == XmlNodeType.EndElement && xtr.Name == "enemies"))
                        {
                            xtr.Read();
                            if (xtr.ReadState == ReadState.EndOfFile)
                            {
                                return;
                            }
                        }

                        if (xtr.NodeType != XmlNodeType.None)
                        {
                            xtr.Read();
                            if (xtr.ReadState == ReadState.EndOfFile)
                            {
                                return;
                            }
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
