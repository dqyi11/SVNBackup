namespace PathPlanner
{
    partial class ToolbarForm
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
            this.tbPlanLength = new System.Windows.Forms.TextBox();
            this.lbPlanLength = new System.Windows.Forms.Label();
            this.rbInfoMax = new System.Windows.Forms.RadioButton();
            this.rbRandom = new System.Windows.Forms.RadioButton();
            this.tbPlan = new System.Windows.Forms.Button();
            this.rbDistMin = new System.Windows.Forms.RadioButton();
            this.cbEnableHumanConstraint = new System.Windows.Forms.CheckBox();
            this.comboAppMode = new System.Windows.Forms.ComboBox();
            this.lbMode = new System.Windows.Forms.Label();
            this.groupBoxPlanning = new System.Windows.Forms.GroupBox();
            this.groupBoxHumanPath = new System.Windows.Forms.GroupBox();
            this.btnE = new System.Windows.Forms.Button();
            this.btnW = new System.Windows.Forms.Button();
            this.btnSE = new System.Windows.Forms.Button();
            this.btnSW = new System.Windows.Forms.Button();
            this.btnStay = new System.Windows.Forms.Button();
            this.btnNE = new System.Windows.Forms.Button();
            this.btnNW = new System.Windows.Forms.Button();
            this.groupBoxPlanning.SuspendLayout();
            this.groupBoxHumanPath.SuspendLayout();
            this.SuspendLayout();
            // 
            // tbPlanLength
            // 
            this.tbPlanLength.Location = new System.Drawing.Point(73, 42);
            this.tbPlanLength.Name = "tbPlanLength";
            this.tbPlanLength.Size = new System.Drawing.Size(100, 20);
            this.tbPlanLength.TabIndex = 0;
            // 
            // lbPlanLength
            // 
            this.lbPlanLength.AutoSize = true;
            this.lbPlanLength.Location = new System.Drawing.Point(3, 42);
            this.lbPlanLength.Name = "lbPlanLength";
            this.lbPlanLength.Size = new System.Drawing.Size(64, 13);
            this.lbPlanLength.TabIndex = 1;
            this.lbPlanLength.Text = "Plan Length";
            // 
            // rbInfoMax
            // 
            this.rbInfoMax.AutoSize = true;
            this.rbInfoMax.Location = new System.Drawing.Point(6, 91);
            this.rbInfoMax.Name = "rbInfoMax";
            this.rbInfoMax.Size = new System.Drawing.Size(100, 17);
            this.rbInfoMax.TabIndex = 2;
            this.rbInfoMax.TabStop = true;
            this.rbInfoMax.Text = "Information Max";
            this.rbInfoMax.UseVisualStyleBackColor = true;
            this.rbInfoMax.CheckedChanged += new System.EventHandler(this.rbInfoMax_CheckedChanged);
            this.rbInfoMax.Click += new System.EventHandler(this.rbInfoMax_Click);
            // 
            // rbRandom
            // 
            this.rbRandom.AutoSize = true;
            this.rbRandom.Location = new System.Drawing.Point(6, 114);
            this.rbRandom.Name = "rbRandom";
            this.rbRandom.Size = new System.Drawing.Size(65, 17);
            this.rbRandom.TabIndex = 3;
            this.rbRandom.TabStop = true;
            this.rbRandom.Text = "Random";
            this.rbRandom.UseVisualStyleBackColor = true;
            this.rbRandom.Click += new System.EventHandler(this.rbRandom_Click);
            // 
            // tbPlan
            // 
            this.tbPlan.Location = new System.Drawing.Point(6, 137);
            this.tbPlan.Name = "tbPlan";
            this.tbPlan.Size = new System.Drawing.Size(75, 23);
            this.tbPlan.TabIndex = 4;
            this.tbPlan.Text = "Plan Path";
            this.tbPlan.UseVisualStyleBackColor = true;
            this.tbPlan.Click += new System.EventHandler(this.tbPlan_Click);
            // 
            // rbDistMin
            // 
            this.rbDistMin.AutoSize = true;
            this.rbDistMin.Location = new System.Drawing.Point(6, 68);
            this.rbDistMin.Name = "rbDistMin";
            this.rbDistMin.Size = new System.Drawing.Size(87, 17);
            this.rbDistMin.TabIndex = 5;
            this.rbDistMin.TabStop = true;
            this.rbDistMin.Text = "Distance Min";
            this.rbDistMin.UseVisualStyleBackColor = true;
            this.rbDistMin.Click += new System.EventHandler(this.rbDistMin_Click);
            // 
            // cbEnableHumanConstraint
            // 
            this.cbEnableHumanConstraint.AutoSize = true;
            this.cbEnableHumanConstraint.Location = new System.Drawing.Point(6, 19);
            this.cbEnableHumanConstraint.Name = "cbEnableHumanConstraint";
            this.cbEnableHumanConstraint.Size = new System.Drawing.Size(146, 17);
            this.cbEnableHumanConstraint.TabIndex = 6;
            this.cbEnableHumanConstraint.Text = "Enable Human Constraint";
            this.cbEnableHumanConstraint.UseVisualStyleBackColor = true;
            // 
            // comboAppMode
            // 
            this.comboAppMode.FormattingEnabled = true;
            this.comboAppMode.Location = new System.Drawing.Point(12, 26);
            this.comboAppMode.Name = "comboAppMode";
            this.comboAppMode.Size = new System.Drawing.Size(191, 21);
            this.comboAppMode.TabIndex = 7;
            this.comboAppMode.SelectedIndexChanged += new System.EventHandler(this.comboAppMode_SelectedIndexChanged);
            // 
            // lbMode
            // 
            this.lbMode.AutoSize = true;
            this.lbMode.Location = new System.Drawing.Point(16, 10);
            this.lbMode.Name = "lbMode";
            this.lbMode.Size = new System.Drawing.Size(42, 13);
            this.lbMode.TabIndex = 8;
            this.lbMode.Text = "MODE:";
            // 
            // groupBoxPlanning
            // 
            this.groupBoxPlanning.Controls.Add(this.cbEnableHumanConstraint);
            this.groupBoxPlanning.Controls.Add(this.tbPlanLength);
            this.groupBoxPlanning.Controls.Add(this.lbPlanLength);
            this.groupBoxPlanning.Controls.Add(this.tbPlan);
            this.groupBoxPlanning.Controls.Add(this.rbDistMin);
            this.groupBoxPlanning.Controls.Add(this.rbRandom);
            this.groupBoxPlanning.Controls.Add(this.rbInfoMax);
            this.groupBoxPlanning.Location = new System.Drawing.Point(12, 167);
            this.groupBoxPlanning.Name = "groupBoxPlanning";
            this.groupBoxPlanning.Size = new System.Drawing.Size(191, 168);
            this.groupBoxPlanning.TabIndex = 9;
            this.groupBoxPlanning.TabStop = false;
            this.groupBoxPlanning.Text = "Planning";
            // 
            // groupBoxHumanPath
            // 
            this.groupBoxHumanPath.Controls.Add(this.btnE);
            this.groupBoxHumanPath.Controls.Add(this.btnW);
            this.groupBoxHumanPath.Controls.Add(this.btnSE);
            this.groupBoxHumanPath.Controls.Add(this.btnSW);
            this.groupBoxHumanPath.Controls.Add(this.btnStay);
            this.groupBoxHumanPath.Controls.Add(this.btnNE);
            this.groupBoxHumanPath.Controls.Add(this.btnNW);
            this.groupBoxHumanPath.Location = new System.Drawing.Point(13, 54);
            this.groupBoxHumanPath.Name = "groupBoxHumanPath";
            this.groupBoxHumanPath.Size = new System.Drawing.Size(190, 107);
            this.groupBoxHumanPath.TabIndex = 10;
            this.groupBoxHumanPath.TabStop = false;
            this.groupBoxHumanPath.Text = "Human Path";
            // 
            // btnE
            // 
            this.btnE.Location = new System.Drawing.Point(127, 49);
            this.btnE.Name = "btnE";
            this.btnE.Size = new System.Drawing.Size(49, 23);
            this.btnE.TabIndex = 6;
            this.btnE.Text = "E";
            this.btnE.UseVisualStyleBackColor = true;
            this.btnE.Click += new System.EventHandler(this.btnE_Click);
            // 
            // btnW
            // 
            this.btnW.Location = new System.Drawing.Point(17, 48);
            this.btnW.Name = "btnW";
            this.btnW.Size = new System.Drawing.Size(49, 23);
            this.btnW.TabIndex = 5;
            this.btnW.Text = "W";
            this.btnW.UseVisualStyleBackColor = true;
            this.btnW.Click += new System.EventHandler(this.btnW_Click);
            // 
            // btnSE
            // 
            this.btnSE.Location = new System.Drawing.Point(98, 78);
            this.btnSE.Name = "btnSE";
            this.btnSE.Size = new System.Drawing.Size(49, 23);
            this.btnSE.TabIndex = 4;
            this.btnSE.Text = "SE";
            this.btnSE.UseVisualStyleBackColor = true;
            this.btnSE.Click += new System.EventHandler(this.btnSE_Click);
            // 
            // btnSW
            // 
            this.btnSW.Location = new System.Drawing.Point(43, 77);
            this.btnSW.Name = "btnSW";
            this.btnSW.Size = new System.Drawing.Size(49, 23);
            this.btnSW.TabIndex = 3;
            this.btnSW.Text = "SW";
            this.btnSW.UseVisualStyleBackColor = true;
            this.btnSW.Click += new System.EventHandler(this.btnSW_Click);
            // 
            // btnStay
            // 
            this.btnStay.Location = new System.Drawing.Point(72, 48);
            this.btnStay.Name = "btnStay";
            this.btnStay.Size = new System.Drawing.Size(49, 23);
            this.btnStay.TabIndex = 2;
            this.btnStay.Text = "STAY";
            this.btnStay.UseVisualStyleBackColor = true;
            this.btnStay.Click += new System.EventHandler(this.btnStay_Click);
            // 
            // btnNE
            // 
            this.btnNE.Location = new System.Drawing.Point(98, 19);
            this.btnNE.Name = "btnNE";
            this.btnNE.Size = new System.Drawing.Size(49, 23);
            this.btnNE.TabIndex = 1;
            this.btnNE.Text = "NE";
            this.btnNE.UseVisualStyleBackColor = true;
            this.btnNE.Click += new System.EventHandler(this.btnNE_Click);
            // 
            // btnNW
            // 
            this.btnNW.Location = new System.Drawing.Point(43, 19);
            this.btnNW.Name = "btnNW";
            this.btnNW.Size = new System.Drawing.Size(49, 23);
            this.btnNW.TabIndex = 0;
            this.btnNW.Text = "NW";
            this.btnNW.UseVisualStyleBackColor = true;
            this.btnNW.Click += new System.EventHandler(this.btnNW_Click);
            // 
            // ToolbarForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(215, 423);
            this.Controls.Add(this.groupBoxHumanPath);
            this.Controls.Add(this.groupBoxPlanning);
            this.Controls.Add(this.lbMode);
            this.Controls.Add(this.comboAppMode);
            this.Name = "ToolbarForm";
            this.Text = "Toobar";
            this.Load += new System.EventHandler(this.ToolbarForm_Load);
            this.groupBoxPlanning.ResumeLayout(false);
            this.groupBoxPlanning.PerformLayout();
            this.groupBoxHumanPath.ResumeLayout(false);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.TextBox tbPlanLength;
        private System.Windows.Forms.Label lbPlanLength;
        private System.Windows.Forms.RadioButton rbInfoMax;
        private System.Windows.Forms.RadioButton rbRandom;
        private System.Windows.Forms.Button tbPlan;
        private System.Windows.Forms.RadioButton rbDistMin;
        private System.Windows.Forms.CheckBox cbEnableHumanConstraint;
        private System.Windows.Forms.ComboBox comboAppMode;
        private System.Windows.Forms.Label lbMode;
        private System.Windows.Forms.GroupBox groupBoxPlanning;
        private System.Windows.Forms.GroupBox groupBoxHumanPath;
        private System.Windows.Forms.Button btnSE;
        private System.Windows.Forms.Button btnSW;
        private System.Windows.Forms.Button btnStay;
        private System.Windows.Forms.Button btnNE;
        private System.Windows.Forms.Button btnNW;
        private System.Windows.Forms.Button btnE;
        private System.Windows.Forms.Button btnW;
    }
}