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

    public partial class EnemyPropertyForm : Form
    {
        EnemyLabel enemyLabel = null;
        EnemyLabelManager labelMgr = null;

        public EnemyPropertyForm()
        {
            InitializeComponent();            
        }

        public EnemyPropertyForm(EnemyLabel point, EnemyLabelManager mgr)
        {
            enemyLabel = point;
            labelMgr = mgr;
            this.Text = "Feature";
            InitializeComponent();
        }

        private void PropertyForm_Load(object sender, EventArgs e)
        {

            this.tbPos.Text = enemyLabel.pos.ToString();
            this.tbName.Text = enemyLabel.name;
            this.tbId.Text = enemyLabel.id;

        }

        private void btnOK_Click(object sender, EventArgs e)
        {
            if (true == labelMgr.IsLabelInList(this.tbId.Text))
            {
                MessageBox.Show("Illegal Id");
                return;
            }
            enemyLabel.id = this.tbId.Text;
            enemyLabel.name = this.tbName.Text;

            labelMgr.AddLabel(enemyLabel);
            this.Close();
        }

        private void btnCancel_Click(object sender, EventArgs e)
        {
            this.Close();
        }
    }
}
