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
    public partial class IndoorPropertyForm : Form
    {
        IndoorLabel indoorLabel = null;
        IndoorLabelManager labelMgr = null;

        public IndoorPropertyForm()
        {
            InitializeComponent();            
        }

        public IndoorPropertyForm(IndoorLabel indoor, IndoorLabelManager mgr)
        {
            this.indoorLabel = indoor;
            this.labelMgr = mgr;
            this.Text = "Indoor";
            InitializeComponent();
        }

        private void PropertyForm_Load(object sender, EventArgs e)
        {

            foreach (Point p in indoorLabel.vertices)
            {
                this.tbPos.Text += p.X.ToString() + "," + p.Y.ToString() + " ";
            }
            this.tbName.Text = indoorLabel.name;
            this.tbId.Text = indoorLabel.id;
            
        }

        private void btnOK_Click(object sender, EventArgs e)
        {
            if (true == labelMgr.IsLabelInList(this.tbId.Text))
            {
                MessageBox.Show("Illegal Id");
                return;
            }

            indoorLabel.id = this.tbId.Text; 
            indoorLabel.name = this.tbName.Text;

            labelMgr.AddLabel(indoorLabel);
            
            this.Close();
        }

        private void btnCancel_Click(object sender, EventArgs e)
        {
            this.Close();
        }
    }
}
