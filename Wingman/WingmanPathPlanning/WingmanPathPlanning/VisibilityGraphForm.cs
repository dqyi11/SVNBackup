using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Hexagonal;


namespace WingmanPathPlanning
{
    public partial class VisibilityGraphForm : Form
    {
        VisibilityGraph _visibilityGraph;
        HexagonalMapDrawer _graphDrawer;

        public VisibilityGraphForm(HexagonalMap map)
        {
            _visibilityGraph = new VisibilityGraph(map);
            _graphDrawer = new HexagonalMapDrawer(_visibilityGraph.VisGraph);
            InitializeComponent();
        }

        protected override void Dispose(bool disposing)
        {
            Hide();
        }

        private void VisibilityGraphForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            e.Cancel = true;
            this.Visible = false;
        }

        private void VisibilityGraphForm_Paint(object sender, PaintEventArgs e)
        {
            //Draw the graphics/GUI
            foreach (Control c in this.Controls)
            {
                c.Refresh();
            }

            if (_graphDrawer != null)
            {
                _graphDrawer.Draw(e.Graphics);
            }
        }
    }
}
