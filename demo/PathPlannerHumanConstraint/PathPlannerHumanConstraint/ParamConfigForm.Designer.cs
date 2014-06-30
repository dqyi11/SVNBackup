namespace PathPlanner
{
    partial class ParamConfigForm
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
            this.cfgTabCtrl = new System.Windows.Forms.TabControl();
            this.generalTab = new System.Windows.Forms.TabPage();
            this.tbHexaSize = new System.Windows.Forms.TextBox();
            this.lblHexaSize = new System.Windows.Forms.Label();
            this.infoMaxTab = new System.Windows.Forms.TabPage();
            this.tbRobotObs = new System.Windows.Forms.TextBox();
            this.lbRobotObs = new System.Windows.Forms.Label();
            this.tbHumanObs = new System.Windows.Forms.TextBox();
            this.lbHumanObs = new System.Windows.Forms.Label();
            this.tbWingman = new System.Windows.Forms.TextBox();
            this.lbWingman = new System.Windows.Forms.Label();
            this.btnOK = new System.Windows.Forms.Button();
            this.btnCancel = new System.Windows.Forms.Button();
            this.cfgTabCtrl.SuspendLayout();
            this.generalTab.SuspendLayout();
            this.infoMaxTab.SuspendLayout();
            this.SuspendLayout();
            // 
            // cfgTabCtrl
            // 
            this.cfgTabCtrl.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left)
                        | System.Windows.Forms.AnchorStyles.Right)));
            this.cfgTabCtrl.Controls.Add(this.generalTab);
            this.cfgTabCtrl.Controls.Add(this.infoMaxTab);
            this.cfgTabCtrl.Location = new System.Drawing.Point(0, 0);
            this.cfgTabCtrl.Name = "cfgTabCtrl";
            this.cfgTabCtrl.SelectedIndex = 0;
            this.cfgTabCtrl.Size = new System.Drawing.Size(479, 254);
            this.cfgTabCtrl.TabIndex = 0;
            // 
            // generalTab
            // 
            this.generalTab.Controls.Add(this.tbHexaSize);
            this.generalTab.Controls.Add(this.lblHexaSize);
            this.generalTab.Location = new System.Drawing.Point(4, 22);
            this.generalTab.Name = "generalTab";
            this.generalTab.Padding = new System.Windows.Forms.Padding(3);
            this.generalTab.Size = new System.Drawing.Size(471, 228);
            this.generalTab.TabIndex = 0;
            this.generalTab.Text = "General";
            this.generalTab.UseVisualStyleBackColor = true;
            // 
            // tbHexaSize
            // 
            this.tbHexaSize.Location = new System.Drawing.Point(65, 1);
            this.tbHexaSize.Name = "tbHexaSize";
            this.tbHexaSize.Size = new System.Drawing.Size(67, 20);
            this.tbHexaSize.TabIndex = 1;
            // 
            // lblHexaSize
            // 
            this.lblHexaSize.AutoSize = true;
            this.lblHexaSize.Location = new System.Drawing.Point(4, 4);
            this.lblHexaSize.Name = "lblHexaSize";
            this.lblHexaSize.Size = new System.Drawing.Size(55, 13);
            this.lblHexaSize.TabIndex = 0;
            this.lblHexaSize.Text = "HexaSize:";
            // 
            // infoMaxTab
            // 
            this.infoMaxTab.Controls.Add(this.tbRobotObs);
            this.infoMaxTab.Controls.Add(this.lbRobotObs);
            this.infoMaxTab.Controls.Add(this.tbHumanObs);
            this.infoMaxTab.Controls.Add(this.lbHumanObs);
            this.infoMaxTab.Controls.Add(this.tbWingman);
            this.infoMaxTab.Controls.Add(this.lbWingman);
            this.infoMaxTab.Location = new System.Drawing.Point(4, 22);
            this.infoMaxTab.Name = "infoMaxTab";
            this.infoMaxTab.Padding = new System.Windows.Forms.Padding(3);
            this.infoMaxTab.Size = new System.Drawing.Size(471, 228);
            this.infoMaxTab.TabIndex = 1;
            this.infoMaxTab.Text = "InfoMax";
            this.infoMaxTab.UseVisualStyleBackColor = true;
            // 
            // tbRobotObs
            // 
            this.tbRobotObs.Location = new System.Drawing.Point(118, 79);
            this.tbRobotObs.Name = "tbRobotObs";
            this.tbRobotObs.Size = new System.Drawing.Size(100, 20);
            this.tbRobotObs.TabIndex = 5;
            // 
            // lbRobotObs
            // 
            this.lbRobotObs.AutoSize = true;
            this.lbRobotObs.Location = new System.Drawing.Point(8, 79);
            this.lbRobotObs.Name = "lbRobotObs";
            this.lbRobotObs.Size = new System.Drawing.Size(97, 13);
            this.lbRobotObs.TabIndex = 4;
            this.lbRobotObs.Text = "Robot observation:";
            // 
            // tbHumanObs
            // 
            this.tbHumanObs.Location = new System.Drawing.Point(118, 45);
            this.tbHumanObs.Name = "tbHumanObs";
            this.tbHumanObs.Size = new System.Drawing.Size(100, 20);
            this.tbHumanObs.TabIndex = 3;
            // 
            // lbHumanObs
            // 
            this.lbHumanObs.AutoSize = true;
            this.lbHumanObs.Location = new System.Drawing.Point(8, 45);
            this.lbHumanObs.Name = "lbHumanObs";
            this.lbHumanObs.Size = new System.Drawing.Size(102, 13);
            this.lbHumanObs.TabIndex = 2;
            this.lbHumanObs.Text = "Human observation:";
            // 
            // tbWingman
            // 
            this.tbWingman.Location = new System.Drawing.Point(118, 12);
            this.tbWingman.Name = "tbWingman";
            this.tbWingman.Size = new System.Drawing.Size(100, 20);
            this.tbWingman.TabIndex = 1;
            // 
            // lbWingman
            // 
            this.lbWingman.AutoSize = true;
            this.lbWingman.Location = new System.Drawing.Point(8, 12);
            this.lbWingman.Name = "lbWingman";
            this.lbWingman.Size = new System.Drawing.Size(104, 13);
            this.lbWingman.TabIndex = 0;
            this.lbWingman.Text = "Wingman constraint:";
            // 
            // btnOK
            // 
            this.btnOK.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.btnOK.Location = new System.Drawing.Point(292, 252);
            this.btnOK.Name = "btnOK";
            this.btnOK.Size = new System.Drawing.Size(75, 25);
            this.btnOK.TabIndex = 1;
            this.btnOK.Text = "OK";
            this.btnOK.UseVisualStyleBackColor = true;
            this.btnOK.Click += new System.EventHandler(this.btnOK_Click);
            // 
            // btnCancel
            // 
            this.btnCancel.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.btnCancel.Location = new System.Drawing.Point(400, 252);
            this.btnCancel.Name = "btnCancel";
            this.btnCancel.Size = new System.Drawing.Size(75, 25);
            this.btnCancel.TabIndex = 2;
            this.btnCancel.Text = "Cancel";
            this.btnCancel.UseVisualStyleBackColor = true;
            this.btnCancel.Click += new System.EventHandler(this.btnCancel_Click);
            // 
            // ParamConfigForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(479, 284);
            this.Controls.Add(this.btnCancel);
            this.Controls.Add(this.btnOK);
            this.Controls.Add(this.cfgTabCtrl);
            this.Name = "ParamConfigForm";
            this.Text = "Config";
            this.Load += new System.EventHandler(this.ParamConfigForm_Load);
            this.cfgTabCtrl.ResumeLayout(false);
            this.generalTab.ResumeLayout(false);
            this.generalTab.PerformLayout();
            this.infoMaxTab.ResumeLayout(false);
            this.infoMaxTab.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.TabControl cfgTabCtrl;
        private System.Windows.Forms.TabPage generalTab;
        private System.Windows.Forms.TabPage infoMaxTab;
        private System.Windows.Forms.Button btnOK;
        private System.Windows.Forms.Button btnCancel;
        private System.Windows.Forms.TextBox tbHexaSize;
        private System.Windows.Forms.Label lblHexaSize;
        private System.Windows.Forms.TextBox tbRobotObs;
        private System.Windows.Forms.Label lbRobotObs;
        private System.Windows.Forms.TextBox tbHumanObs;
        private System.Windows.Forms.Label lbHumanObs;
        private System.Windows.Forms.TextBox tbWingman;
        private System.Windows.Forms.Label lbWingman;
    }
}