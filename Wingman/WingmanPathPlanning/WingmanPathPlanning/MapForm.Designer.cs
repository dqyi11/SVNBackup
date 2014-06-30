namespace WingmanPathPlanning
{
    partial class MapForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.btnW = new System.Windows.Forms.Button();
            this.btnNW = new System.Windows.Forms.Button();
            this.btnNE = new System.Windows.Forms.Button();
            this.btnStay = new System.Windows.Forms.Button();
            this.btnE = new System.Windows.Forms.Button();
            this.btnSW = new System.Windows.Forms.Button();
            this.btnSE = new System.Windows.Forms.Button();
            this.HCtrl = new System.Windows.Forms.GroupBox();
            this.modeCtrl = new System.Windows.Forms.GroupBox();
            this.rbBatchTest = new System.Windows.Forms.RadioButton();
            this.btnSimTest = new System.Windows.Forms.Button();
            this.btnVisGraph = new System.Windows.Forms.Button();
            this.btnAnimation = new System.Windows.Forms.Button();
            this.btnRobotPath = new System.Windows.Forms.Button();
            this.btnHumanPath = new System.Windows.Forms.Button();
            this.tbHumanPath = new System.Windows.Forms.TextBox();
            this.lbHumanPath = new System.Windows.Forms.Label();
            this.gbParam = new System.Windows.Forms.GroupBox();
            this.lbRbObsFt = new System.Windows.Forms.Label();
            this.lbHmObsFt = new System.Windows.Forms.Label();
            this.tbRbObsFt = new System.Windows.Forms.TextBox();
            this.tbHmObsFt = new System.Windows.Forms.TextBox();
            this.tbRR = new System.Windows.Forms.TextBox();
            this.lbRR = new System.Windows.Forms.Label();
            this.tbHR = new System.Windows.Forms.TextBox();
            this.lbHR = new System.Windows.Forms.Label();
            this.tbWR = new System.Windows.Forms.TextBox();
            this.lbWingmanRadius = new System.Windows.Forms.Label();
            this.gbEnv = new System.Windows.Forms.GroupBox();
            this.btnLoadEnv = new System.Windows.Forms.Button();
            this.btnPrtEnv = new System.Windows.Forms.Button();
            this.btnResetEnv = new System.Windows.Forms.Button();
            this.btnRndEnv = new System.Windows.Forms.Button();
            this.btnClearObstacle = new System.Windows.Forms.Button();
            this.btnObstacle = new System.Windows.Forms.Button();
            this.openFileDialog1 = new System.Windows.Forms.OpenFileDialog();
            this.HCtrl.SuspendLayout();
            this.modeCtrl.SuspendLayout();
            this.gbParam.SuspendLayout();
            this.gbEnv.SuspendLayout();
            this.SuspendLayout();
            // 
            // btnW
            // 
            this.btnW.Location = new System.Drawing.Point(6, 40);
            this.btnW.Name = "btnW";
            this.btnW.Size = new System.Drawing.Size(48, 24);
            this.btnW.TabIndex = 2;
            this.btnW.Text = "W";
            this.btnW.UseVisualStyleBackColor = true;
            this.btnW.Click += new System.EventHandler(this.btnW_Click);
            // 
            // btnNW
            // 
            this.btnNW.Location = new System.Drawing.Point(33, 10);
            this.btnNW.Name = "btnNW";
            this.btnNW.Size = new System.Drawing.Size(48, 24);
            this.btnNW.TabIndex = 3;
            this.btnNW.Text = "NW";
            this.btnNW.UseVisualStyleBackColor = true;
            this.btnNW.Click += new System.EventHandler(this.btnNW_Click);
            // 
            // btnNE
            // 
            this.btnNE.Location = new System.Drawing.Point(87, 10);
            this.btnNE.Name = "btnNE";
            this.btnNE.Size = new System.Drawing.Size(48, 24);
            this.btnNE.TabIndex = 4;
            this.btnNE.Text = "NE";
            this.btnNE.UseVisualStyleBackColor = true;
            this.btnNE.Click += new System.EventHandler(this.btnNE_Click);
            // 
            // btnStay
            // 
            this.btnStay.Location = new System.Drawing.Point(60, 40);
            this.btnStay.Name = "btnStay";
            this.btnStay.Size = new System.Drawing.Size(48, 24);
            this.btnStay.TabIndex = 5;
            this.btnStay.Text = "Stay";
            this.btnStay.UseVisualStyleBackColor = true;
            this.btnStay.Click += new System.EventHandler(this.btnStay_Click);
            // 
            // btnE
            // 
            this.btnE.Location = new System.Drawing.Point(114, 40);
            this.btnE.Name = "btnE";
            this.btnE.Size = new System.Drawing.Size(48, 24);
            this.btnE.TabIndex = 6;
            this.btnE.Text = "E";
            this.btnE.UseVisualStyleBackColor = true;
            this.btnE.Click += new System.EventHandler(this.btnE_Click);
            // 
            // btnSW
            // 
            this.btnSW.Location = new System.Drawing.Point(33, 70);
            this.btnSW.Name = "btnSW";
            this.btnSW.Size = new System.Drawing.Size(48, 24);
            this.btnSW.TabIndex = 7;
            this.btnSW.Text = "SW";
            this.btnSW.UseVisualStyleBackColor = true;
            this.btnSW.Click += new System.EventHandler(this.btnSW_Click);
            // 
            // btnSE
            // 
            this.btnSE.Location = new System.Drawing.Point(87, 70);
            this.btnSE.Name = "btnSE";
            this.btnSE.Size = new System.Drawing.Size(48, 24);
            this.btnSE.TabIndex = 8;
            this.btnSE.Text = "SE";
            this.btnSE.UseVisualStyleBackColor = true;
            this.btnSE.Click += new System.EventHandler(this.btnSE_Click);
            // 
            // HCtrl
            // 
            this.HCtrl.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.HCtrl.Controls.Add(this.btnNW);
            this.HCtrl.Controls.Add(this.btnSE);
            this.HCtrl.Controls.Add(this.btnNE);
            this.HCtrl.Controls.Add(this.btnSW);
            this.HCtrl.Controls.Add(this.btnW);
            this.HCtrl.Controls.Add(this.btnE);
            this.HCtrl.Controls.Add(this.btnStay);
            this.HCtrl.Enabled = false;
            this.HCtrl.Location = new System.Drawing.Point(818, 336);
            this.HCtrl.Name = "HCtrl";
            this.HCtrl.Size = new System.Drawing.Size(171, 97);
            this.HCtrl.TabIndex = 9;
            this.HCtrl.TabStop = false;
            this.HCtrl.Text = "Ctrl";
            // 
            // modeCtrl
            // 
            this.modeCtrl.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.modeCtrl.Controls.Add(this.rbBatchTest);
            this.modeCtrl.Controls.Add(this.btnSimTest);
            this.modeCtrl.Controls.Add(this.btnVisGraph);
            this.modeCtrl.Controls.Add(this.btnAnimation);
            this.modeCtrl.Controls.Add(this.btnRobotPath);
            this.modeCtrl.Controls.Add(this.btnHumanPath);
            this.modeCtrl.Location = new System.Drawing.Point(817, 439);
            this.modeCtrl.Name = "modeCtrl";
            this.modeCtrl.Size = new System.Drawing.Size(171, 213);
            this.modeCtrl.TabIndex = 10;
            this.modeCtrl.TabStop = false;
            this.modeCtrl.Text = "Mode";
            // 
            // rbBatchTest
            // 
            this.rbBatchTest.AutoSize = true;
            this.rbBatchTest.Location = new System.Drawing.Point(9, 178);
            this.rbBatchTest.Name = "rbBatchTest";
            this.rbBatchTest.Size = new System.Drawing.Size(77, 17);
            this.rbBatchTest.TabIndex = 5;
            this.rbBatchTest.Text = "Batch Test";
            this.rbBatchTest.UseVisualStyleBackColor = true;
            // 
            // btnSimTest
            // 
            this.btnSimTest.Location = new System.Drawing.Point(8, 115);
            this.btnSimTest.Name = "btnSimTest";
            this.btnSimTest.Size = new System.Drawing.Size(154, 25);
            this.btnSimTest.TabIndex = 4;
            this.btnSimTest.Text = "Sim Test";
            this.btnSimTest.UseVisualStyleBackColor = true;
            this.btnSimTest.Click += new System.EventHandler(this.btnSimTest_Click);
            // 
            // btnVisGraph
            // 
            this.btnVisGraph.Location = new System.Drawing.Point(7, 146);
            this.btnVisGraph.Name = "btnVisGraph";
            this.btnVisGraph.Size = new System.Drawing.Size(153, 25);
            this.btnVisGraph.TabIndex = 3;
            this.btnVisGraph.Text = "Show Visibility Graph";
            this.btnVisGraph.UseVisualStyleBackColor = true;
            this.btnVisGraph.Click += new System.EventHandler(this.btnVisGraph_Click);
            // 
            // btnAnimation
            // 
            this.btnAnimation.Location = new System.Drawing.Point(6, 52);
            this.btnAnimation.Name = "btnAnimation";
            this.btnAnimation.Size = new System.Drawing.Size(155, 25);
            this.btnAnimation.TabIndex = 2;
            this.btnAnimation.Text = "Show Search Space";
            this.btnAnimation.UseVisualStyleBackColor = true;
            this.btnAnimation.Click += new System.EventHandler(this.btnAnimation_Click);
            // 
            // btnRobotPath
            // 
            this.btnRobotPath.Location = new System.Drawing.Point(6, 83);
            this.btnRobotPath.Name = "btnRobotPath";
            this.btnRobotPath.Size = new System.Drawing.Size(156, 25);
            this.btnRobotPath.TabIndex = 1;
            this.btnRobotPath.Text = "Generate Robot Path";
            this.btnRobotPath.UseVisualStyleBackColor = true;
            this.btnRobotPath.Click += new System.EventHandler(this.btnRobotPath_Click);
            // 
            // btnHumanPath
            // 
            this.btnHumanPath.Location = new System.Drawing.Point(5, 22);
            this.btnHumanPath.Name = "btnHumanPath";
            this.btnHumanPath.Size = new System.Drawing.Size(156, 25);
            this.btnHumanPath.TabIndex = 0;
            this.btnHumanPath.Text = "Generate Human Path";
            this.btnHumanPath.UseVisualStyleBackColor = true;
            this.btnHumanPath.Click += new System.EventHandler(this.btnHumanPath_Click);
            // 
            // tbHumanPath
            // 
            this.tbHumanPath.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.tbHumanPath.BackColor = System.Drawing.SystemColors.ButtonHighlight;
            this.tbHumanPath.Location = new System.Drawing.Point(818, 671);
            this.tbHumanPath.Multiline = true;
            this.tbHumanPath.Name = "tbHumanPath";
            this.tbHumanPath.ReadOnly = true;
            this.tbHumanPath.ScrollBars = System.Windows.Forms.ScrollBars.Both;
            this.tbHumanPath.Size = new System.Drawing.Size(180, 141);
            this.tbHumanPath.TabIndex = 11;
            // 
            // lbHumanPath
            // 
            this.lbHumanPath.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.lbHumanPath.AutoSize = true;
            this.lbHumanPath.Location = new System.Drawing.Point(814, 655);
            this.lbHumanPath.Name = "lbHumanPath";
            this.lbHumanPath.Size = new System.Drawing.Size(66, 13);
            this.lbHumanPath.TabIndex = 12;
            this.lbHumanPath.Text = "Human Path";
            // 
            // gbParam
            // 
            this.gbParam.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.gbParam.Controls.Add(this.lbRbObsFt);
            this.gbParam.Controls.Add(this.lbHmObsFt);
            this.gbParam.Controls.Add(this.tbRbObsFt);
            this.gbParam.Controls.Add(this.tbHmObsFt);
            this.gbParam.Controls.Add(this.tbRR);
            this.gbParam.Controls.Add(this.lbRR);
            this.gbParam.Controls.Add(this.tbHR);
            this.gbParam.Controls.Add(this.lbHR);
            this.gbParam.Controls.Add(this.tbWR);
            this.gbParam.Controls.Add(this.lbWingmanRadius);
            this.gbParam.Location = new System.Drawing.Point(817, 7);
            this.gbParam.Name = "gbParam";
            this.gbParam.Size = new System.Drawing.Size(171, 138);
            this.gbParam.TabIndex = 13;
            this.gbParam.TabStop = false;
            this.gbParam.Text = "Parameter";
            // 
            // lbRbObsFt
            // 
            this.lbRbObsFt.AutoSize = true;
            this.lbRbObsFt.Location = new System.Drawing.Point(6, 121);
            this.lbRbObsFt.Name = "lbRbObsFt";
            this.lbRbObsFt.Size = new System.Drawing.Size(58, 13);
            this.lbRbObsFt.TabIndex = 9;
            this.lbRbObsFt.Text = "Rb Obs Ft:";
            // 
            // lbHmObsFt
            // 
            this.lbHmObsFt.AutoSize = true;
            this.lbHmObsFt.Location = new System.Drawing.Point(6, 98);
            this.lbHmObsFt.Name = "lbHmObsFt";
            this.lbHmObsFt.Size = new System.Drawing.Size(60, 13);
            this.lbHmObsFt.TabIndex = 8;
            this.lbHmObsFt.Text = "Hm Obs Ft:";
            // 
            // tbRbObsFt
            // 
            this.tbRbObsFt.Location = new System.Drawing.Point(104, 118);
            this.tbRbObsFt.Name = "tbRbObsFt";
            this.tbRbObsFt.Size = new System.Drawing.Size(58, 20);
            this.tbRbObsFt.TabIndex = 7;
            this.tbRbObsFt.TextChanged += new System.EventHandler(this.tbRbObsFt_TextChanged);
            // 
            // tbHmObsFt
            // 
            this.tbHmObsFt.Location = new System.Drawing.Point(104, 89);
            this.tbHmObsFt.Name = "tbHmObsFt";
            this.tbHmObsFt.Size = new System.Drawing.Size(58, 20);
            this.tbHmObsFt.TabIndex = 6;
            this.tbHmObsFt.TextChanged += new System.EventHandler(this.tbHmObsFt_TextChanged);
            // 
            // tbRR
            // 
            this.tbRR.Location = new System.Drawing.Point(104, 63);
            this.tbRR.Name = "tbRR";
            this.tbRR.Size = new System.Drawing.Size(58, 20);
            this.tbRR.TabIndex = 5;
            this.tbRR.TextChanged += new System.EventHandler(this.tbRR_TextChanged);
            // 
            // lbRR
            // 
            this.lbRR.AutoSize = true;
            this.lbRR.Location = new System.Drawing.Point(3, 73);
            this.lbRR.Name = "lbRR";
            this.lbRR.Size = new System.Drawing.Size(69, 13);
            this.lbRR.TabIndex = 4;
            this.lbRR.Text = "Rb Obs Rng:";
            // 
            // tbHR
            // 
            this.tbHR.Location = new System.Drawing.Point(104, 39);
            this.tbHR.Name = "tbHR";
            this.tbHR.Size = new System.Drawing.Size(58, 20);
            this.tbHR.TabIndex = 3;
            this.tbHR.TextChanged += new System.EventHandler(this.tbHR_TextChanged);
            // 
            // lbHR
            // 
            this.lbHR.AutoSize = true;
            this.lbHR.Location = new System.Drawing.Point(3, 49);
            this.lbHR.Name = "lbHR";
            this.lbHR.Size = new System.Drawing.Size(71, 13);
            this.lbHR.TabIndex = 2;
            this.lbHR.Text = "Hm Obs Rng:";
            // 
            // tbWR
            // 
            this.tbWR.Location = new System.Drawing.Point(104, 13);
            this.tbWR.Name = "tbWR";
            this.tbWR.Size = new System.Drawing.Size(58, 20);
            this.tbWR.TabIndex = 1;
            this.tbWR.TextChanged += new System.EventHandler(this.tbWR_TextChanged);
            // 
            // lbWingmanRadius
            // 
            this.lbWingmanRadius.AutoSize = true;
            this.lbWingmanRadius.Location = new System.Drawing.Point(3, 23);
            this.lbWingmanRadius.Name = "lbWingmanRadius";
            this.lbWingmanRadius.Size = new System.Drawing.Size(91, 13);
            this.lbWingmanRadius.TabIndex = 0;
            this.lbWingmanRadius.Text = "Wingman Radius:";
            // 
            // gbEnv
            // 
            this.gbEnv.Controls.Add(this.btnLoadEnv);
            this.gbEnv.Controls.Add(this.btnPrtEnv);
            this.gbEnv.Controls.Add(this.btnResetEnv);
            this.gbEnv.Controls.Add(this.btnRndEnv);
            this.gbEnv.Controls.Add(this.btnClearObstacle);
            this.gbEnv.Controls.Add(this.btnObstacle);
            this.gbEnv.Location = new System.Drawing.Point(817, 145);
            this.gbEnv.Name = "gbEnv";
            this.gbEnv.Size = new System.Drawing.Size(171, 185);
            this.gbEnv.TabIndex = 14;
            this.gbEnv.TabStop = false;
            this.gbEnv.Text = "Env";
            // 
            // btnLoadEnv
            // 
            this.btnLoadEnv.Location = new System.Drawing.Point(6, 156);
            this.btnLoadEnv.Name = "btnLoadEnv";
            this.btnLoadEnv.Size = new System.Drawing.Size(152, 23);
            this.btnLoadEnv.TabIndex = 5;
            this.btnLoadEnv.Text = "Load Environment";
            this.btnLoadEnv.UseVisualStyleBackColor = true;
            this.btnLoadEnv.Click += new System.EventHandler(this.btnLoadEnv_Click);
            // 
            // btnPrtEnv
            // 
            this.btnPrtEnv.Location = new System.Drawing.Point(6, 125);
            this.btnPrtEnv.Name = "btnPrtEnv";
            this.btnPrtEnv.Size = new System.Drawing.Size(152, 25);
            this.btnPrtEnv.TabIndex = 4;
            this.btnPrtEnv.Text = "Print Environment";
            this.btnPrtEnv.UseVisualStyleBackColor = true;
            this.btnPrtEnv.Click += new System.EventHandler(this.btnPrtEnv_Click);
            // 
            // btnResetEnv
            // 
            this.btnResetEnv.Location = new System.Drawing.Point(5, 98);
            this.btnResetEnv.Name = "btnResetEnv";
            this.btnResetEnv.Size = new System.Drawing.Size(154, 25);
            this.btnResetEnv.TabIndex = 3;
            this.btnResetEnv.Text = "Reset Environment";
            this.btnResetEnv.UseVisualStyleBackColor = true;
            this.btnResetEnv.Click += new System.EventHandler(this.btnResetEnv_Click);
            // 
            // btnRndEnv
            // 
            this.btnRndEnv.Location = new System.Drawing.Point(5, 70);
            this.btnRndEnv.Name = "btnRndEnv";
            this.btnRndEnv.Size = new System.Drawing.Size(154, 25);
            this.btnRndEnv.TabIndex = 2;
            this.btnRndEnv.Text = "Randomize Environment";
            this.btnRndEnv.UseVisualStyleBackColor = true;
            this.btnRndEnv.Click += new System.EventHandler(this.btnRndEnv_Click);
            // 
            // btnClearObstacle
            // 
            this.btnClearObstacle.Location = new System.Drawing.Point(5, 43);
            this.btnClearObstacle.Name = "btnClearObstacle";
            this.btnClearObstacle.Size = new System.Drawing.Size(154, 25);
            this.btnClearObstacle.TabIndex = 1;
            this.btnClearObstacle.Text = "Clear Obstacles";
            this.btnClearObstacle.UseVisualStyleBackColor = true;
            this.btnClearObstacle.Click += new System.EventHandler(this.btnClearObstacle_Click);
            // 
            // btnObstacle
            // 
            this.btnObstacle.Location = new System.Drawing.Point(5, 16);
            this.btnObstacle.Name = "btnObstacle";
            this.btnObstacle.Size = new System.Drawing.Size(154, 25);
            this.btnObstacle.TabIndex = 0;
            this.btnObstacle.Text = "Edit Obstacle";
            this.btnObstacle.UseVisualStyleBackColor = true;
            this.btnObstacle.Click += new System.EventHandler(this.btnObstacle_Click);
            // 
            // openFileDialog1
            // 
            this.openFileDialog1.FileName = "openFileDialog1";
            // 
            // MapForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1000, 813);
            this.Controls.Add(this.gbEnv);
            this.Controls.Add(this.gbParam);
            this.Controls.Add(this.lbHumanPath);
            this.Controls.Add(this.tbHumanPath);
            this.Controls.Add(this.modeCtrl);
            this.Controls.Add(this.HCtrl);
            this.DoubleBuffered = true;
            this.Name = "MapForm";
            this.Text = "Wingman Path Planning";
            this.Load += new System.EventHandler(this.MapForm_Load);
            this.Paint += new System.Windows.Forms.PaintEventHandler(this.MapForm_Paint);
            this.MouseClick += new System.Windows.Forms.MouseEventHandler(this.MapForm_MouseClick);
            this.HCtrl.ResumeLayout(false);
            this.modeCtrl.ResumeLayout(false);
            this.modeCtrl.PerformLayout();
            this.gbParam.ResumeLayout(false);
            this.gbParam.PerformLayout();
            this.gbEnv.ResumeLayout(false);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button btnW;
        private System.Windows.Forms.Button btnNW;
        private System.Windows.Forms.Button btnNE;
        private System.Windows.Forms.Button btnStay;
        private System.Windows.Forms.Button btnE;
        private System.Windows.Forms.Button btnSW;
        private System.Windows.Forms.Button btnSE;
        private System.Windows.Forms.GroupBox HCtrl;
        private System.Windows.Forms.GroupBox modeCtrl;
        private System.Windows.Forms.Button btnRobotPath;
        private System.Windows.Forms.Button btnHumanPath;
        private System.Windows.Forms.TextBox tbHumanPath;
        private System.Windows.Forms.Label lbHumanPath;
        private System.Windows.Forms.GroupBox gbParam;
        private System.Windows.Forms.Label lbWingmanRadius;
        private System.Windows.Forms.TextBox tbWR;
        private System.Windows.Forms.TextBox tbRR;
        private System.Windows.Forms.Label lbRR;
        private System.Windows.Forms.TextBox tbHR;
        private System.Windows.Forms.Label lbHR;
        private System.Windows.Forms.Button btnAnimation;
        private System.Windows.Forms.GroupBox gbEnv;
        private System.Windows.Forms.Button btnObstacle;
        private System.Windows.Forms.Button btnClearObstacle;
        private System.Windows.Forms.Button btnVisGraph;
        private System.Windows.Forms.Button btnRndEnv;
        private System.Windows.Forms.Button btnResetEnv;
        private System.Windows.Forms.Button btnPrtEnv;
        private System.Windows.Forms.Label lbHmObsFt;
        private System.Windows.Forms.TextBox tbRbObsFt;
        private System.Windows.Forms.TextBox tbHmObsFt;
        private System.Windows.Forms.Label lbRbObsFt;
        private System.Windows.Forms.Button btnSimTest;
        private System.Windows.Forms.Button btnLoadEnv;
        private System.Windows.Forms.OpenFileDialog openFileDialog1;
        private System.Windows.Forms.RadioButton rbBatchTest;

    }
}

