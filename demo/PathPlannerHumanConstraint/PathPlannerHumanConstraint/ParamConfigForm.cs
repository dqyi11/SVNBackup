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

        public string wingmanConstraint
        {
            get
            {
                return this.tbWingman.Text;
            }
        }

        public string humanObservation
        {
            get
            {
                return this.tbHumanObs.Text;
            }
        }

        public string robotObservation
        {
            get
            {
                return this.tbRobotObs.Text;
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
            this.tbWingman.Text = paramMgr.wingmanConstraint.ToString();
            this.tbHumanObs.Text = paramMgr.humanObs.ToString();
            this.tbRobotObs.Text = paramMgr.humanObs.ToString();
        }

        private void btnCancel_Click(object sender, EventArgs e)
        {
            //this.Close();
            this.Hide();
        }

    }
}
