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

namespace WingmanPathPlanning
{
    enum FormState { IDLE = 1, EDIT_OBSTACLE, ADD_HUMAN_INIT, GENERATING_HUMANPATH, SHOWING_SEARCHSPACE,
    HUMANPATH_GENERATED
    };

    public partial class MapForm : Form
    {
        HexagonalMap _map;
        HexagonalMapDrawer _mapDrawer;
        ParameterManager _paramMgr;

        Timer transitTimer;
        int transitCurrentCnt;

        Human _human;
        Robot _robot;
     
        FormState _formState;

        VisibilityGraphForm _visGraphForm;
        PlanningForm _planningForm;

        public HexagonalMap map
        {
            get { return _map;  }
        }

        public HexaPath HumanPath
        {
            get
            {
                return _human.path;
            }
        }

        public Human human
        {
            get
            {
                return _human;
            }
        }


        public MapForm(HexagonalMap map, ParameterManager paramMgr)
        {
            _map = map;
            _mapDrawer = new HexagonalMapDrawer(_map, 20, 20);
            _paramMgr = paramMgr;

            _formState = FormState.IDLE;

            _human = new Human(_map);
            _robot = new Robot(_map);

            InitializeComponent();

            transitTimer = new Timer();
            transitTimer.Tick += new EventHandler(this.ShowSearchSpace);

            transitCurrentCnt = 0;
            _visGraphForm = new VisibilityGraphForm(_map);
        }

        private void MapForm_Load(object sender, EventArgs e)
        {
            this.Size = new Size(Width, Height);

            this.tbWR.Text = _paramMgr.wingmanRadius.ToString();
            this.tbHR.Text = _paramMgr.humanObservation.ToString();
            this.tbRR.Text = _paramMgr.robotObservation.ToString();
            this.tbHmObsFt.Text = _paramMgr.humanObservationFactor.ToString();
            this.tbRbObsFt.Text = _paramMgr.robotObservationFactor.ToString();
            _visGraphForm.Hide();

            _human.WingmanToleranceRange = int.Parse(tbWR.Text);
            _human.SetObservationRange(int.Parse(tbHR.Text));
            _robot.SetObservationRange(int.Parse(tbRR.Text));
            _human.confidenceFactor = double.Parse(tbHmObsFt.Text);
            _robot.confidenceFactor = double.Parse(tbRbObsFt.Text);
        }

        private void MapForm_Paint(object sender, PaintEventArgs e)
        {

            this.btnAnimation.Text = "Show Search Space";
            this.btnHumanPath.Text = "Generate Human Path";
            this.btnObstacle.Text = "Edit Obstacles";
            this.btnRobotPath.Enabled = false;
            this.btnHumanPath.Enabled = true;
            this.btnAnimation.Enabled = true;
            this.btnClearObstacle.Enabled = false;
            this.HCtrl.Enabled = false;

            switch (_formState)
            {
                case FormState.IDLE:
                default:
                    break;
                case FormState.EDIT_OBSTACLE:
                    this.btnObstacle.Enabled = true;
                    this.btnObstacle.Text = "Finish Editing Obstacles";
                    this.btnClearObstacle.Enabled = true;
                    this.btnHumanPath.Enabled = false;
                    this.btnAnimation.Enabled = false;
                    this.btnRobotPath.Enabled = false;
                    break;
                case FormState.ADD_HUMAN_INIT:
                    this.btnRobotPath.Enabled = false;
                    this.btnHumanPath.Text = "Finish Human Path";
                    this.btnAnimation.Enabled = false;
                    
                    break;
                case FormState.GENERATING_HUMANPATH:
                    this.btnRobotPath.Enabled = false;
                    this.btnHumanPath.Text = "Finish Human Path";
                    this.btnAnimation.Enabled = false;
                    this.HCtrl.Enabled = true;
                    break;
                case FormState.HUMANPATH_GENERATED:
                    this.btnRobotPath.Enabled = true;
                    break;
                case FormState.SHOWING_SEARCHSPACE:
                    this.btnAnimation.Text = "Quit";
                    break;
            }
            
            //update textbox
            if (_human != null)
            {
                int humanPathLength = _human.path.Length;

                if (humanPathLength > 0)
                {
                    HexSet humanHexSet = _map.MapState.GetHexSet(_human.hexSetIdx);
                    this.tbHumanPath.Clear();
                    for (int i = 0; i < humanPathLength; i++)
                    {
                        HexaPos pos = _human.path[i];
                        string posInfo = i.ToString() + " : " + pos.X.ToString() + " , " + pos.Y.ToString() + "\n";
                        this.tbHumanPath.AppendText(posInfo);
                        humanHexSet.hexSet.Add(_map.GetHex(pos.X, pos.Y));
                    }
                }
                else
                {
                    this.tbHumanPath.Clear();
                    if (_map.MapState.GetHexSetNum() > 0)
                    {
                        HexSet humanHexSet = _map.MapState.GetHexSet(_human.hexSetIdx);
                        humanHexSet.hexSet.Clear();
                    }
                }

            }

            //Draw the graphics/GUI
            foreach (Control c in this.Controls)
            {
                c.Refresh();
            }

            if (_mapDrawer != null)
            {
                _mapDrawer.Draw(e.Graphics);
            }

            //Force the next Paint()
            //this.Invalidate();

        }

        private void MapForm_Closing(object sender, FormClosingEventArgs e)
        {
            if (_mapDrawer != null)
            {
                _mapDrawer = null;
            }

            if (_map != null)
            {
                _map = null;
            }
        }

        private void MapForm_MouseClick(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                if (_formState == FormState.ADD_HUMAN_INIT)
                {
                    Point mouseClick = new Point(e.X - _mapDrawer.BoardXOffset, e.Y - _mapDrawer.BoardYOffset);
                    Hex clickedHex = _map.FindHexMouseClick(mouseClick);

                    if (false == _map.MapState.IsObstacle(clickedHex))
                    {
                        _human.SetInitPos(new HexaPos(clickedHex.posX, clickedHex.posY));

                        _formState = FormState.GENERATING_HUMANPATH;
                        _map.MapState.ActiveHex = clickedHex;

                        _human.hexSetIdx = _map.MapState.CreateHexSet();
                        HexSet hexSet = _map.MapState.GetHexSet(_human.hexSetIdx);
                        hexSet.borderColor = Color.Blue;
                    }
                }
                else if (_formState == FormState.EDIT_OBSTACLE)
                {
                    Point mouseClick = new Point(e.X - _mapDrawer.BoardXOffset, e.Y - _mapDrawer.BoardYOffset);
                    Hex clickedHex = _map.FindHexMouseClick(mouseClick);

                    _map.MapState.AddObstalce(clickedHex);
                }
            }
            Refresh();
        }

        private void btnHumanPath_Click(object sender, EventArgs e)
        {
            if (_formState == FormState.IDLE)
            {
                _formState = FormState.ADD_HUMAN_INIT;
            }
            else if (_formState == FormState.GENERATING_HUMANPATH)
            {
                _formState = FormState.HUMANPATH_GENERATED;

                if (this.rbBatchTest.Checked == false)
                {
                    _map.GetMapStateMgr().Update(_human, _human.path);
                }
            }
            else if(_formState == FormState.ADD_HUMAN_INIT)
            {
                _formState = FormState.IDLE;
                // reset 
                _map.MapState.ActiveHex = null;
                _human.path.Clear();
            }
            else if (_formState == FormState.HUMANPATH_GENERATED)
            {
                if (MessageBox.Show("Clear Human Path?", "Warning", MessageBoxButtons.OKCancel) == DialogResult.OK)
                {
                    _map.MapState.ActiveHex = null;
                    _human.path.Clear();
                    _formState = FormState.ADD_HUMAN_INIT;

                }
            }

            Refresh();
        }

        private void btnW_Click(object sender, EventArgs e)
        {
            if (_formState == FormState.GENERATING_HUMANPATH)
            {
                _human.Move(HexagonalMap.Direction.WEST);
                Refresh();
            }            
        }

        private void btnSW_Click(object sender, EventArgs e)
        {
            if (_formState == FormState.GENERATING_HUMANPATH)
            {
                _human.Move(HexagonalMap.Direction.SW);
                Refresh();
            }            
        }

        private void btnNW_Click(object sender, EventArgs e)
        {
            if (_formState == FormState.GENERATING_HUMANPATH)
            {
                _human.Move(HexagonalMap.Direction.NW);
                Refresh();
            }            
        }

        private void btnStay_Click(object sender, EventArgs e)
        {
            if (_formState == FormState.GENERATING_HUMANPATH)
            {
                _human.Move(HexagonalMap.Direction.STAY);
                Refresh();
            }            
        }

        private void btnE_Click(object sender, EventArgs e)
        {
            if (_formState == FormState.GENERATING_HUMANPATH)
            {
                _human.Move(HexagonalMap.Direction.EAST);
                Refresh();
            }
        }

        private void btnNE_Click(object sender, EventArgs e)
        {
            if (_formState == FormState.GENERATING_HUMANPATH)
            {
                _human.Move(HexagonalMap.Direction.NE);
                Refresh();
            }            
        }

        private void btnSE_Click(object sender, EventArgs e)
        {
            if (_formState == FormState.GENERATING_HUMANPATH)
            {
                _human.Move(HexagonalMap.Direction.SE);
                Refresh();
            }            
        }

        private void btnRobotPath_Click(object sender, EventArgs e)
        {
            if (this.rbBatchTest.Checked == true)
            {
                //ParameterTester tester = new ParameterTester(this);
                //GdParameterTester tester = new GdParameterTester(this);
                TeleportTester tester = new TeleportTester(this);
                tester.Run();
                MessageBox.Show("Finished batch test");
            }
            else
            {
                // apply parameters
                _human.WingmanToleranceRange = int.Parse(tbWR.Text);
                _human.SetObservationRange(int.Parse(tbHR.Text));

                TopologyGraphGenerator topologyGenerator = new TopologyGraphGenerator(_map);

                TopologyGraph tograph = topologyGenerator.GetTopologyGraph();
                //graph.Print();
                tograph.Draw();

                PlanningGraphGenerator planningGenerator = new PlanningGraphGenerator(tograph);
                PathPlanningGraph planGraph = planningGenerator.GetPathPlanningGraph(this.HumanPath, this.human.WingmanToleranceRange);

                planGraph.Draw();

                HexaPos startPos = _human.path[0];

                PlanningGraphPruner pruner = new PlanningGraphPruner();
                PathPlanningGraph newPlanGraph = pruner.GetPlanningGraph(planGraph, startPos);

                newPlanGraph.Draw("PlanningGraph2");

                PathPlanningGraph prunedPlanGraph = pruner.BackwardPrune(newPlanGraph);
                prunedPlanGraph = pruner.ForwardPrune(prunedPlanGraph);

                prunedPlanGraph.Draw("PrunedPlanningGraph");

                /*
                ExhaustiveDFSPathPlanner planner = new ExhaustiveDFSPathPlanner(_map, _robot);
                HexaPath maxPath = planner.FindPath(prunedPlanGraph, startPos);

                Console.WriteLine("PATH BY EXHAUSTIVE DFS: " + maxPath.ToString());
                 */

                _planningForm = new PlanningForm(prunedPlanGraph, _map, _robot, startPos, _mapDrawer, _human.path);
                _planningForm.Show();
            }
        }

        private void btnAnimation_Click(object sender, EventArgs e)
        {
            if (_formState != FormState.SHOWING_SEARCHSPACE)
            {
                transitTimer.Interval = 1000;
                transitTimer.Start();
                _formState = FormState.SHOWING_SEARCHSPACE;

                // reset count
                transitCurrentCnt = 0;

                _human.WingmanToleranceRange = _paramMgr.wingmanRadius;
            }
            else
            {
                transitTimer.Stop();
                _formState = FormState.HUMANPATH_GENERATED;
                _map.MapState.ActiveHexSet = null;

                Refresh();
            }

        }

        private void ShowSearchSpace(object sender, EventArgs e)
        {
            if (_formState == FormState.SHOWING_SEARCHSPACE)
            {
                if (transitCurrentCnt >= _human.path.Length)
                {
                    transitCurrentCnt = 0;
                }

                HexaPos currentPos = _human.path[transitCurrentCnt];

                List<HexaPos> neighbors = _map.GetHexes(currentPos.X, currentPos.Y, _human.WingmanToleranceRange, true);

                _map.MapState.ActiveHexSet = _map.Convert(neighbors);

                Refresh();

                transitCurrentCnt++;
            }
        }


        private void btnObstacle_Click(object sender, EventArgs e)
        {
            if (_formState == FormState.EDIT_OBSTACLE)
            {
                _formState = FormState.IDLE;
            }
            else
            {
                _formState = FormState.EDIT_OBSTACLE;
            }

            Refresh();
        }

        private void btnClearObstacle_Click(object sender, EventArgs e)
        {
            if (_formState == FormState.EDIT_OBSTACLE)
            {
                _map.MapState.ClearObstacle();

                Refresh();
            }
        }

        private void btnVisGraph_Click(object sender, EventArgs e)
        {          
            _visGraphForm.Show();           
        }

        private void btnRndEnv_Click(object sender, EventArgs e)
        {
            _map.GetMapStateMgr().RandomizeValue();

            Refresh();
        }

        private void tbWR_TextChanged(object sender, EventArgs e)
        {
            try
            {
                Convert.ToInt64(tbWR.Text);
            }
            catch
            {
                MessageBox.Show("Wrong Format");
                tbWR.Undo();
                tbWR.ClearUndo();
            }
            _human.WingmanToleranceRange = int.Parse(tbWR.Text);
        }

        private void tbHR_TextChanged(object sender, EventArgs e)
        {
            try
            {
                Convert.ToInt64(tbHR.Text);
            }
            catch
            {
                MessageBox.Show("Wrong Format");
                tbHR.Undo();
                tbHR.ClearUndo();
            }
            _human.SetObservationRange(int.Parse(tbHR.Text));
        }

        private void tbRR_TextChanged(object sender, EventArgs e)
        {
            try
            {
                Convert.ToInt64(tbRR.Text);
            }
            catch
            {
                MessageBox.Show("Wrong Format");
                tbRR.Undo();
                tbRR.ClearUndo();
            }
            _robot.SetObservationRange(int.Parse(tbRR.Text));
        }

        private void btnResetEnv_Click(object sender, EventArgs e)
        {
            if (MessageBox.Show("Reset Environment?", "Warning", MessageBoxButtons.OKCancel) == DialogResult.OK)
            {
                _map.GetMapStateMgr().Reset();
                Refresh();
            }
        }

        private void btnPrtEnv_Click(object sender, EventArgs e)
        {
            string filename = "ENV-" + DateTime.Now.ToShortTimeString() + ".png";
            filename = filename.Replace(":", "-");
            _mapDrawer.DrawEnv(filename, _map.GetMapStateMgr().CopyEntropy(), _human.path, Color.Blue, null);
        }

        private void tbHmObsFt_TextChanged(object sender, EventArgs e)
        {
            try
            {
                Convert.ToDouble(tbHmObsFt.Text);
            }
            catch
            {
                MessageBox.Show("Wrong Format");
                tbHmObsFt.Undo();
                tbHmObsFt.ClearUndo();
            }

            if (double.Parse(tbHmObsFt.Text) < 0 || double.Parse(tbHmObsFt.Text) > 1)
            {
                MessageBox.Show("Wrong Range");
                tbHmObsFt.Undo();
                tbHmObsFt.ClearUndo();
            }
            _human.confidenceFactor = double.Parse(tbHmObsFt.Text);
        }

        private void tbRbObsFt_TextChanged(object sender, EventArgs e)
        {
            try
            {
                Convert.ToDouble(tbRbObsFt.Text);
            }
            catch
            {
                MessageBox.Show("Wrong Format");
                tbRbObsFt.Undo();
                tbRbObsFt.ClearUndo();
            }

            if (double.Parse(tbRbObsFt.Text) < 0 || double.Parse(tbRbObsFt.Text) > 1)
            {
                MessageBox.Show("Wrong Range");
                tbRbObsFt.Undo();
                tbRbObsFt.ClearUndo();
            }
            _robot.confidenceFactor = double.Parse(tbRbObsFt.Text);
        }

        private void btnSimTest_Click(object sender, EventArgs e)
        {
            SimTestForm simForm = new SimTestForm(_paramMgr);
            simForm.Show();
        }

        private void btnLoadEnv_Click(object sender, EventArgs e)
        {
            openFileDialog1.Filter = "XML Files|*.xml";
            if (openFileDialog1.ShowDialog() == System.Windows.Forms.DialogResult.OK)
            {
                EnvLoader envLoader = new EnvLoader();
                envLoader.Load(openFileDialog1.FileName);
                double[,] newProb = envLoader.CalcVal(_map);
                for(int i=0;i<_map.Height;i++)
                {
                    for(int j=0;j<_map.Width;j++)
                    {
                        _map.GetMapStateMgr().SetEntropy(i, j, 1.0 - newProb[i, j]);
                    }
                }
                Refresh();                
            }
        }

    }
        
}
