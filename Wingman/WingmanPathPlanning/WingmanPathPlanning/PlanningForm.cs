using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Planner;
using WingmanPathPlanning.Data;
using WingmanPathPlanning.Hexagonal;
using WingmanPathPlanning.Base;
using System.Threading;

namespace WingmanPathPlanning
{
    public partial class PlanningForm : Form
    {
        PathPlanningGraph _graph;
        HexagonalMap _map;
        HexagonalMapDrawer _mapDrawer;
        Agent _agent;
        HexaPos _startPos;
        HexaPath _humanPath;
        public enum planMethod { UNKNOWN, EX_DFS, ITERATIVE_BACK_PROP, GENE_ALG, SIMPLE_GREEDY,
                                 BACK_PROP, IT_BACK_PROP_ENH, IT_BACK_PROP_RETRK, IT_BP_COMBO,
                                 EXPAND_TREE_1, EXPAND_TREE_N}
        planMethod[] planMethodList = { planMethod.EX_DFS, planMethod.ITERATIVE_BACK_PROP, 
                                          planMethod.GENE_ALG, planMethod.SIMPLE_GREEDY,
                                          planMethod.BACK_PROP, planMethod.IT_BACK_PROP_ENH,
                                          planMethod.IT_BACK_PROP_RETRK, planMethod.IT_BP_COMBO,
                                          planMethod.EXPAND_TREE_1, planMethod.EXPAND_TREE_N};
        string[] methods = { "Exhaustive DFS", "Iterative Backward Propagation", "Genetic Algorithm", "Simple Greedy", 
                               "Backward Propagation", "Iterative Backward Propagation Enhanced",
                               "Iterative Backward Propagation Retrack", "Iterative Backward Propagtion Combo",
                               "Expanding Tree 1", "Expanding Tree N"};

        planMethod _planMethod;

        public PlanningForm(PathPlanningGraph graph, HexagonalMap map, Agent agent, HexaPos startPos, HexagonalMapDrawer mapDrawer, HexaPath humanPath = null)
        {
            _graph = graph;
            _map = map;
            _mapDrawer = mapDrawer;
            _agent = agent;
            _startPos = startPos;
            _humanPath = humanPath;
            _planMethod = planMethod.UNKNOWN;
            InitializeComponent();
        }

        private void PlanningForm_Load(object sender, EventArgs e)
        {
            cbMethodSelect.BeginUpdate(); 
            for(int i=0;i < methods.Length;i++)
            {
                cbMethodSelect.Items.Add(methods[i]);
            }
            cbMethodSelect.EndUpdate();
        }

        private void cbMethodSelect_SelectedIndexChanged(object sender, EventArgs e)
        {
            for (int i = 0; i < methods.Length; i++)
            {
                if (cbMethodSelect.SelectedItem.ToString() == methods[i])
                {
                    _planMethod = planMethodList[i];
                }
            }
            Refresh();
        }

        private void btnStart_Click(object sender, EventArgs e)
        {
            string methodName = "";
            string methodShortName = "";
            long startTime = 0;
            long endTime = 0;
            long deltaTime = 0;
            double spendTime = 0.0;
            HexaPath maxPath = new HexaPath();

            startTime = DateTime.Now.Ticks;
            if (_planMethod == planMethod.EX_DFS)
            {
                ExhaustiveDFSPathPlanner planner = new ExhaustiveDFSPathPlanner(_map, (Robot)_agent);
                maxPath = planner.FindPath(_graph, _startPos);
                methodName = "EXHAUSTIVE DFS";
                methodShortName = "EXDFS";

            }
            else if (_planMethod == planMethod.ITERATIVE_BACK_PROP)
            {
                IterativeBackPropPathPlanner planner = new IterativeBackPropPathPlanner(_map, (Robot)_agent);
                maxPath = planner.FindPath(_graph, _startPos);
                methodName = "ITERATIVE BACK PROPAGATE";
                methodShortName = "ITBP";
            }
            else if (_planMethod == planMethod.SIMPLE_GREEDY)
            {
                SimpleGreedyPathPlanner planner = new SimpleGreedyPathPlanner(_map, (Robot)_agent);
                maxPath = planner.FindPath(_graph, _startPos);
                methodName = "SIMPLE GREEDY";
                methodShortName = "SG";
            }
            else if (_planMethod == planMethod.GENE_ALG)
            {
                GeneticAlgorithmPathPlanner planner = new GeneticAlgorithmPathPlanner(_map, (Robot)_agent);
                maxPath = planner.FindPath(_graph, _startPos);
                methodName = "GENETIC ALGORITHM";
                methodShortName = "GA";
            }
            else if (_planMethod == planMethod.BACK_PROP)
            {
                BackPropPathPlanner planner = new BackPropPathPlanner(_map, (Robot)_agent);
                maxPath = planner.FindPath(_graph, _startPos);
                methodName = "BACK PROP";
                methodShortName = "BP";
            }
            else if (_planMethod == planMethod.IT_BACK_PROP_ENH)
            {
                IterativeBackPropEnhPathPlanner planner = new IterativeBackPropEnhPathPlanner(_map, (Robot)_agent);
                maxPath = planner.FindPath(_graph, _startPos);
                methodName = "ITERATIVE BACK PROPAGATE ENH";
                methodShortName = "ITBPE";
            }
            else if (_planMethod == planMethod.IT_BACK_PROP_RETRK)
            {
                IterativeBackPropRetrackPathPlanner planner = new IterativeBackPropRetrackPathPlanner(_map, (Robot)_agent);
                maxPath = planner.FindPath(_graph, _startPos);
                methodName = "ITERATIVE BACK PROPAGATE RETRK";
                methodShortName = "ITBPR";
            }
            else if (_planMethod == planMethod.IT_BP_COMBO)
            {
                IterativeBackPropComboPathPlanner planner = new IterativeBackPropComboPathPlanner(_map, (Robot)_agent);
                maxPath = planner.FindPath(_graph, _startPos);
                methodName = "ITERTATIVE BACK PROP COMBO";
                methodShortName = "ITBPCOM";
            }
            else if (_planMethod == planMethod.EXPAND_TREE_1)
            {
                TreeExpandingWithIterativeTrackingPathPlanner planner
                    = new TreeExpandingWithIterativeTrackingPathPlanner(_map, (Robot)_agent);
                planner.iteratingOnce = true;
                maxPath = planner.FindPath(_graph, _startPos);

                //Console.WriteLine("EXCLUSIVE EXPANDING NODE NUM: " + planner.GetExclusiveExpandingNodeNum(_graph, _startPos));
                methodName = "EXPANDING TREE ONE";
                methodShortName = "EXP_TREE_1";
            }
            else if (_planMethod == planMethod.EXPAND_TREE_N)
            {
                TreeExpandingWithIterativeTrackingPathPlanner planner
                    = new TreeExpandingWithIterativeTrackingPathPlanner(_map, (Robot)_agent);
                planner.iteratingOnce = false;
                maxPath = planner.FindPath(_graph, _startPos);

                // Console.WriteLine("EXCLUSIVE EXPANDING NODE NUM: " + planner.GetExclusiveExpandingNodeNum(_graph, _startPos));
                methodName = "EXPANDING TREE N";
                methodShortName = "EXP_TREE_N";
            }

            endTime = DateTime.Now.Ticks;
            deltaTime = endTime - startTime;
            spendTime = deltaTime / 10000.0;

            Console.WriteLine("PATH BY " + methodName + " : " + maxPath.ToString());
            txtBoxInfo.AppendText("PATH BY " + methodName + " : " + maxPath.ToString() + "\n");
            double[,] tempEntropy = (double[,])_map.GetMapStateMgr().CopyEntropy();
            double maxScore = _agent.Score(maxPath, tempEntropy);
            Console.WriteLine("SCORE: " + maxScore);
            txtBoxInfo.AppendText("SCORE: " + maxScore + "\n");
            Console.WriteLine("TIME SPENT: " + spendTime + "\n");
            txtBoxInfo.AppendText("TIME SPENT: " + spendTime + " ms\n\n");

            _agent.Update(maxPath, tempEntropy);
            string filename =  methodShortName + "-" + DateTime.Now.ToShortTimeString() + ".png";
            filename = filename.Replace(":", "-");
            _mapDrawer.DrawEnv(filename, tempEntropy, maxPath, Color.Green, _humanPath);
        }

        private void PlanningForm_Paint(object sender, PaintEventArgs e)
        {
            if (_planMethod == planMethod.UNKNOWN)
            {
                btnStart.Enabled = false;
                btnApply.Enabled = false;
                btnPrint.Enabled = false;
            }
            else
            {
                btnStart.Enabled = true;
                btnApply.Enabled = true;
                btnPrint.Enabled = true;
            }
            btnStart.Refresh();
            btnApply.Refresh();
            btnPrint.Refresh();
        }
    }
}
