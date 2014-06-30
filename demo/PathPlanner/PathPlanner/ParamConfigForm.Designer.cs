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
            this.infoMaxTab = new System.Windows.Forms.TabPage();
            this.btnOK = new System.Windows.Forms.Button();
            this.btnCancel = new System.Windows.Forms.Button();
            this.lblHexaSize = new System.Windows.Forms.Label();
            this.tbHexaSize = new System.Windows.Forms.TextBox();
            this.cfgTabCtrl.SuspendLayout();
            this.generalTab.SuspendLayout();
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
            this.cfgTabCtrl.Size = new System.Drawing.Size(479, 234);
            this.cfgTabCtrl.TabIndex = 0;
            // 
            // generalTab
            // 
            this.generalTab.Controls.Add(this.tbHexaSize);
            this.generalTab.Controls.Add(this.lblHexaSize);
            this.generalTab.Location = new System.Drawing.Point(4, 22);
            this.generalTab.Name = "generalTab";
            this.generalTab.Padding = new System.Windows.Forms.Padding(3);
            this.generalTab.Size = new System.Drawing.Size(471, 208);
            this.generalTab.TabIndex = 0;
            this.generalTab.Text = "General";
            this.generalTab.UseVisualStyleBackColor = true;
            // 
            // infoMaxTab
            // 
            this.infoMaxTab.Location = new System.Drawing.Point(4, 22);
            this.infoMaxTab.Name = "infoMaxTab";
            this.infoMaxTab.Padding = new System.Windows.Forms.Padding(3);
            this.infoMaxTab.Size = new System.Drawing.Size(471, 208);
            this.infoMaxTab.TabIndex = 1;
            this.infoMaxTab.Text = "InfoMax";
            this.infoMaxTab.UseVisualStyleBackColor = true;
            // 
            // btnOK
            // 
            this.btnOK.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.btnOK.Location = new System.Drawing.Point(292, 233);
            this.btnOK.Name = "btnOK";
            this.btnOK.Size = new System.Drawing.Size(75, 23);
            this.btnOK.TabIndex = 1;
            this.btnOK.Text = "OK";
            this.btnOK.UseVisualStyleBackColor = true;
            this.btnOK.Click += new System.EventHandler(this.btnOK_Click);
            // 
            // btnCancel
            // 
            this.btnCancel.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.btnCancel.Location = new System.Drawing.Point(400, 233);
            this.btnCancel.Name = "btnCancel";
            this.btnCancel.Size = new System.Drawing.Size(75, 23);
            this.btnCancel.TabIndex = 2;
            this.btnCancel.Text = "Cancel";
            this.btnCancel.UseVisualStyleBackColor = true;
            this.btnCancel.Click += new System.EventHandler(this.btnCancel_Click);
            // 
            // lblHexaSize
            // 
            this.lblHexaSize.AutoSize = true;
            this.lblHexaSize.Location = new System.Drawing.Point(4, 4);
            this.lblHexaSize.Name = "lblHexaSize";
            this.lblHexaSize.Size = new System.Drawing.Size(59, 12);
            this.lblHexaSize.TabIndex = 0;
            this.lblHexaSize.Text = "HexaSize:";
            // 
            // tbHexaSize
            // 
            this.tbHexaSize.Location = new System.Drawing.Point(70, 0);
            this.tbHexaSize.Name = "tbHexaSize";
            this.tbHexaSize.Size = new System.Drawing.Size(67, 21);
            this.tbHexaSize.TabIndex = 1;
            // 
            // ParamConfigForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(479, 262);
            this.Controls.Add(this.btnCancel);
            this.Controls.Add(this.btnOK);
            this.Controls.Add(this.cfgTabCtrl);
            this.Name = "ParamConfigForm";
            this.Text = "Config";
            this.Load += new System.EventHandler(this.ParamConfigForm_Load);
            this.cfgTabCtrl.ResumeLayout(false);
            this.generalTab.ResumeLayout(false);
            this.generalTab.PerformLayout();
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
    }
}