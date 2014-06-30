using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using PathPlanner.Data;

namespace PathPlanner
{
    public partial class ParamConfigForm : Form
    {
        private ParamMgr paramMgr;

        public string hexagonalSize
        {
            get
            {
                return this.tbHexaSize.Text;
            }
        }

        public ParamConfigForm(ParamMgr mgr)
        {
            this.paramMgr = mgr;
            InitializeComponent();
        }

        private void btnOK_Click(object sender, EventArgs e)
        {
            this.paramMgr.Save();
            //this.Close();
            this.Hide();

        }

        private void ParamConfigForm_Load(object sender, EventArgs e)
        {
            this.tbHexaSize.Text = paramMgr.hexagonalSize.ToString();
        }

        private void btnCancel_Click(object sender, EventArgs e)
        {
            //this.Close();
            this.Hide();
        }
    }
}
