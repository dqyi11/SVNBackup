using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace MapLabeling
{
    public partial class ToolbarForm : Form
    {
        public MapViewerForm parentForm = null;

        public ToolbarForm()
        {
            InitializeComponent();
        }

        private void Point_CheckedChanged(object sender, EventArgs e)
        {
            if (this.checkboxPoint.Checked == true)
            {
                this.checkboxIndoor.Checked = false;
                this.checkboxOutdoor.Checked = false;
                this.checkboxEnemy.Checked = false;
                this.parentForm.SetFormState(MapViewerForm.FormState.ADD_FEATURE);
            }
            else
            {
                this.parentForm.SetFormState(MapViewerForm.FormState.NORMAL);
            }
        }

        private void checkboxPolygon_CheckedChanged(object sender, EventArgs e)
        {
            if (this.checkboxIndoor.Checked == true)
            {
                this.checkboxPoint.Checked = false;
                this.checkboxOutdoor.Checked = false;
                this.checkboxEnemy.Checked = false;
                this.parentForm.SetFormState(MapViewerForm.FormState.ADD_INDOOR);
            }
            else
            {
                this.parentForm.SetFormState(MapViewerForm.FormState.NORMAL);
            }
        }

        private void checkBoxOutdoor_CheckedChanged(object sender, EventArgs e)
        {
            if (this.checkboxOutdoor.Checked == true)
            {
                this.checkboxIndoor.Checked = false;
                this.checkboxPoint.Checked = false;
                this.checkboxEnemy.Checked = false;
                this.parentForm.SetFormState(MapViewerForm.FormState.ADD_OUTDOOR);
            }
            else
            {
                this.parentForm.SetFormState(MapViewerForm.FormState.NORMAL);
            }
        }

        private void checkboxEnemy_CheckedChanged(object sender, EventArgs e)
        {
            if (this.checkboxEnemy.Checked == true)
            {
                this.checkboxIndoor.Checked = false;
                this.checkboxPoint.Checked = false;
                this.checkboxOutdoor.Checked = false;
                this.parentForm.SetFormState(MapViewerForm.FormState.ADD_ENEMY);
            }
            else
            {
                this.parentForm.SetFormState(MapViewerForm.FormState.NORMAL);
            }
        }

        /*
        private void ToolbarForm_Paint(object sender, PaintEventArgs e)
        {
            e.Graphics.FillRectangle(new SolidBrush(Color.Red), new Rectangle(100, 100, 20, 20));

        }
         */

    }
}
