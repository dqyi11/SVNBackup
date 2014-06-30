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

    public partial class FeaturePropertyForm : Form
    {
        FeatureLabel featureLabel = null;
        FeatureLabelManager labelMgr = null;

        public FeaturePropertyForm()
        {
            InitializeComponent();            
        }

        public FeaturePropertyForm(FeatureLabel point, FeatureLabelManager mgr)
        {
            featureLabel = point;
            labelMgr = mgr;
            this.Text = "Feature";
            InitializeComponent();
        }

        private void PropertyForm_Load(object sender, EventArgs e)
        {

            this.tbPos.Text = featureLabel.pos.ToString();
            this.tbName.Text = featureLabel.name;
            this.tbId.Text = featureLabel.id;

            cbType.BeginUpdate();
            for (int i = 0; i < featureLabel.TYPE_STR.Length; i++)
            {
                cbType.Items.Add(featureLabel.TYPE_STR[i]);
            }

            cbType.SelectedIndex = (int)featureLabel.type;
            cbType.EndUpdate();

        }

        private void btnOK_Click(object sender, EventArgs e)
        {
            if (true == labelMgr.IsLabelInList(this.tbId.Text))
            {
                MessageBox.Show("Illegal Id");
                return;
            }
            featureLabel.id = this.tbId.Text;
            featureLabel.name = this.tbName.Text;
            featureLabel.type = (FeatureLabel.FEATURE_TYPE)this.cbType.SelectedIndex;

            labelMgr.AddLabel(featureLabel);
            this.Close();
        }

        private void btnCancel_Click(object sender, EventArgs e)
        {
            this.Close();
        }
    }
}
