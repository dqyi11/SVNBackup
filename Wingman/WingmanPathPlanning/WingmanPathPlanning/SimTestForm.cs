using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using WingmanPathPlanning.Data;
using WingmanPathPlanning.Hexagonal;
using WingmanPathPlanning.Base;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Planner;
using WingmanPathPlanning.Ext;

namespace WingmanPathPlanning
{
    public partial class SimTestForm : Form
    {
        HexagonalMap _map;
        HexagonalMapDrawer _mapDrawer;
        ParameterManager _paramMgr;
        Human _human;
        Robot _robot;

        double[,] _entropyData;

        PlanningForm.planMethod _planMethod;

        PlanningForm.planMethod[] planMethodList = { PlanningForm.planMethod.EX_DFS, 
                                                       PlanningForm.planMethod.ITERATIVE_BACK_PROP,
                                                       PlanningForm.planMethod.SIMPLE_GREEDY
                                                    };
        string[] methods = { "Exhaustive DFS", "Iterative Backward Propagation", "Simple Greedy"};

        public SimTestForm(ParameterManager paramMgr)
        {
            _paramMgr = paramMgr;
            _map = new HexagonalMap(paramMgr.simTestWidth, paramMgr.simTestHeight, paramMgr.simTestSize, Hexagonal.HexOrientation.Pointy);
            _mapDrawer = new HexagonalMapDrawer(_map,5, 5);
            _human = new Human(_map);
            _robot = new Robot(_map);
            _planMethod = PlanningForm.planMethod.UNKNOWN;

            InitializeComponent();
        }

        private void SimTestForm_Paint(object sender, PaintEventArgs e)
        {
            if (_planMethod == PlanningForm.planMethod.UNKNOWN)
            {
                btnPlanStep.Enabled = false;
            }
            else
            {
                btnPlanStep.Enabled = true;
            }

            if (_mapDrawer != null)
            {
                _mapDrawer.Draw(e.Graphics);
            }

            //Draw the graphics/GUI
            foreach (Control c in this.Controls)
            {
                c.Refresh();
            }
        }

        private void SimTestForm_Load(object sender, EventArgs e)
        {
            _entropyData = new double[_map.Height,_map.Width];
            LoadData();
            mapDataGridView.DataSource = new ArrayDataView(_entropyData);
            mapDataGridView.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.DisplayedCells;

            _human.WingmanToleranceRange = _paramMgr.wingmanRadius;
            _human.SetObservationRange(_paramMgr.humanObservation);
            _robot.SetObservationRange(_paramMgr.robotObservation);
            _human.confidenceFactor = _paramMgr.humanObservationFactor;
            _robot.confidenceFactor = _paramMgr.robotObservationFactor;

            comboBoxMtdSel.BeginUpdate();
            for (int i = 0; i < methods.Length; i++)
            {
                comboBoxMtdSel.Items.Add(methods[i]);
            }
            comboBoxMtdSel.EndUpdate();
        }

        private void mapDataGridView_CellEndEdit(object sender, DataGridViewCellEventArgs e)
        {
            CorrectData();
            DumpData();
            Refresh();
        }

        private void LoadData()
        {
            for (int i = 0; i < _map.Width; i++)
            {
                for (int j = 0; j < _map.Height; j++)
                {
                    _entropyData[j, i] = _map.GetMapStateMgr().GetEntropy(i, j);
                }
            }
        }

        private void DumpData()
        {
            for (int i = 0; i < _map.Width; i++)
            {
                for (int j = 0; j < _map.Height; j++)
                {
                    _map.GetMapStateMgr().SetEntropy(i, j,_entropyData[j, i]);
                }
            }
        }

        private void CorrectData()
        {
            for (int i = 0; i < _map.Width; i++)
            {
                for (int j = 0; j < _map.Height; j++)
                {
                    if (_entropyData[j, i] > 1)
                    {
                        _entropyData[j, i] = 1;
                    }
                    if (_entropyData[j, i] < 0)
                    {
                        _entropyData[j, i] = 0;
                    }
                }
            }
        }

        private bool ValidateData()
        {
            for (int i = 0; i < _map.Width; i++)
            {
                for (int j = 0; j < _map.Height; j++)
                {
                    if (_entropyData[j, i] > 1 || _entropyData[j, i] < 0)
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        private void SimTestForm_MouseClick(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Right)
            {
                Point mouseClick = new Point(e.X - _mapDrawer.BoardXOffset, e.Y - _mapDrawer.BoardYOffset);
                Hex clickedHex = _map.FindHexMouseClick(mouseClick);

                //_map.MapState.ActiveHex = clickedHex;

                if (clickedHex != null)
                {
                    if (_human.path.Length == 0)
                    {
                        _human.hexSetIdx = _map.MapState.CreateHexSet();
                    }

                    _human.path.AddPos(new HexaPos(clickedHex.posX, clickedHex.posY));

                    HexSet hexSet = _map.MapState.GetHexSet(_human.hexSetIdx);
                    hexSet.borderColor = Color.Blue;
                    hexSet.hexSet.Add(clickedHex);
                }
            }
            else
            {
                _map.MapState.ActiveHex = null;
            }
            Refresh();
        }

        private void btnClear_Click(object sender, EventArgs e)
        {
            HexSet hexSet = _map.MapState.GetHexSet(_human.hexSetIdx);
            if (hexSet != null)
            {
                hexSet.Clear();
            }
            _human.path.Clear();
            Refresh();
        }

        private void btnApply_Click(object sender, EventArgs e)
        {
            _map.GetMapStateMgr().Update(_human, _human.path);
            LoadData();
            Refresh();
        }

        private void comboBoxMtdSel_SelectedIndexChanged(object sender, EventArgs e)
        {
            for (int i = 0; i < methods.Length; i++)
            {
                if (comboBoxMtdSel.SelectedItem.ToString() == methods[i])
                {
                    _planMethod = planMethodList[i];
                }
            }
        }

    }
}
