using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using MapLabeling.data;

namespace MapLabeling
{
    public partial class OutdoorPropertyForm : Form
    {
        OutdoorLabel outdoorLabel = null;
        OutdoorLabelManager labelMgr = null;

        public OutdoorPropertyForm()
        {
            InitializeComponent();            
        }

        public OutdoorPropertyForm(OutdoorLabel outdoor, OutdoorLabelManager mgr)
        {
            outdoorLabel = outdoor;
            this.labelMgr = mgr;
            this.Text = "Outdoor";
            InitializeComponent();
        }

        private void PropertyForm_Load(object sender, EventArgs e)
        {
            foreach (Point p in outdoorLabel.vertices)
            {
                this.tbPos.Text += p.X.ToString() + "," + p.Y.ToString() + " ";
            }
            this.tbName.Text = outdoorLabel.name;
            this.tbId.Text = outdoorLabel.id;

            cbType.BeginUpdate();
            for (int i = 0; i < outdoorLabel.TYPE_STR.Length; i++)
            {
                cbType.Items.Add(outdoorLabel.TYPE_STR[i]);
            }

            cbType.SelectedIndex = (int)outdoorLabel.type;
            cbType.EndUpdate();
        }

        private void btnOK_Click(object sender, EventArgs e)
        {
            if (true == labelMgr.IsLabelInList(this.tbId.Text))
            {
                MessageBox.Show("Illegal Id");
                return;
            }

            outdoorLabel.id = this.tbId.Text;
            outdoorLabel.name = this.tbName.Text;
            outdoorLabel.type = (OutdoorLabel.OUTDOOR_TYPE)this.cbType.SelectedIndex;

            labelMgr.AddLabel(outdoorLabel);
            this.Close();
        }

        private void btnCancel_Click(object sender, EventArgs e)
        {
            this.Close();
        }
    }
}
