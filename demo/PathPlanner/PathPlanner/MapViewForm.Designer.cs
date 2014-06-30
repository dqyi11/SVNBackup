namespace PathPlanner
{
    partial class MapViewForm
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
            this.components = new System.ComponentModel.Container();
            this.mapViewMenuStrip = new System.Windows.Forms.MenuStrip();
            this.fileToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.openToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.saveToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.configToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.exitToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.viewToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.toobarToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.mapViewToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.mapToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.hexagonToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.mapViewPanel = new System.Windows.Forms.Panel();
            this.mapViewPictureBox = new System.Windows.Forms.PictureBox();
            this.mapViewOpenFileDialog = new System.Windows.Forms.OpenFileDialog();
            this.mapViewSaveFileDialog = new System.Windows.Forms.SaveFileDialog();
            this.toolbarForm = new PathPlanner.ToolbarForm(this);
			this.cmsAddPoint = new System.Windows.Forms.ContextMenuStrip(this.components);
            this.startToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.endToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.mapViewMenuStrip.SuspendLayout();
            this.mapViewPanel.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.mapViewPictureBox)).BeginInit();
            this.cmsAddPoint.SuspendLayout();
            this.SuspendLayout();
            // 
            // mapViewMenuStrip
            // 
            this.mapViewMenuStrip.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.fileToolStripMenuItem,
            this.viewToolStripMenuItem});
            this.mapViewMenuStrip.Location = new System.Drawing.Point(0, 0);
            this.mapViewMenuStrip.Name = "mapViewMenuStrip";
            this.mapViewMenuStrip.Size = new System.Drawing.Size(494, 25);
            this.mapViewMenuStrip.TabIndex = 0;
            this.mapViewMenuStrip.Text = "menu";
            // 
            // fileToolStripMenuItem
            // 
            this.fileToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.openToolStripMenuItem,
            this.saveToolStripMenuItem,
            this.configToolStripMenuItem,
            this.exitToolStripMenuItem});
            this.fileToolStripMenuItem.Name = "fileToolStripMenuItem";
            this.fileToolStripMenuItem.Size = new System.Drawing.Size(39, 21);
            this.fileToolStripMenuItem.Text = "File";
            // 
            // openToolStripMenuItem
            // 
            this.openToolStripMenuItem.Name = "openToolStripMenuItem";
            this.openToolStripMenuItem.Size = new System.Drawing.Size(114, 22);
            this.openToolStripMenuItem.Text = "Open";
            this.openToolStripMenuItem.Click += new System.EventHandler(this.openToolStripMenuItem_Click);
            // 
            // saveToolStripMenuItem
            // 
            this.saveToolStripMenuItem.Name = "saveToolStripMenuItem";
            this.saveToolStripMenuItem.Size = new System.Drawing.Size(114, 22);
            this.saveToolStripMenuItem.Text = "Save";
            this.saveToolStripMenuItem.Click += new System.EventHandler(this.saveToolStripMenuItem_Click);
            // 
            // configToolStripMenuItem
            // 
            this.configToolStripMenuItem.Name = "configToolStripMenuItem";
            this.configToolStripMenuItem.Size = new System.Drawing.Size(114, 22);
            this.configToolStripMenuItem.Text = "Config";
            this.configToolStripMenuItem.Click += new System.EventHandler(this.configToolStripMenuItem_Click);
            // 
            // exitToolStripMenuItem
            // 
            this.exitToolStripMenuItem.Name = "exitToolStripMenuItem";
            this.exitToolStripMenuItem.Size = new System.Drawing.Size(114, 22);
            this.exitToolStripMenuItem.Text = "Exit";
            this.exitToolStripMenuItem.Click += new System.EventHandler(this.exitToolStripMenuItem_Click);
            // 
            // viewToolStripMenuItem
            // 
            this.viewToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.toobarToolStripMenuItem,
            this.mapViewToolStripMenuItem});
            this.viewToolStripMenuItem.Name = "viewToolStripMenuItem";
            this.viewToolStripMenuItem.Size = new System.Drawing.Size(47, 21);
            this.viewToolStripMenuItem.Text = "View";
            // 
            // toobarToolStripMenuItem
            // 
            this.toobarToolStripMenuItem.Name = "toobarToolStripMenuItem";
            this.toobarToolStripMenuItem.Size = new System.Drawing.Size(130, 22);
            this.toobarToolStripMenuItem.Text = "Toobar";
            this.toobarToolStripMenuItem.Click += new System.EventHandler(this.toobarToolStripMenuItem_Click);
            // 
            // mapViewToolStripMenuItem
            // 
            this.mapViewToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.mapToolStripMenuItem,
            this.hexagonToolStripMenuItem});
            this.mapViewToolStripMenuItem.Name = "mapViewToolStripMenuItem";
            this.mapViewToolStripMenuItem.Size = new System.Drawing.Size(130, 22);
            this.mapViewToolStripMenuItem.Text = "MapView";
            // 
            // mapToolStripMenuItem
            // 
            this.mapToolStripMenuItem.Name = "mapToolStripMenuItem";
            this.mapToolStripMenuItem.Size = new System.Drawing.Size(128, 22);
            this.mapToolStripMenuItem.Text = "Map";
            this.mapToolStripMenuItem.Click += new System.EventHandler(this.mapToolStripMenuItem_Click);
            // 
            // hexagonToolStripMenuItem
            // 
            this.hexagonToolStripMenuItem.Name = "hexagonToolStripMenuItem";
            this.hexagonToolStripMenuItem.Size = new System.Drawing.Size(128, 22);
            this.hexagonToolStripMenuItem.Text = "Hexagon";
            this.hexagonToolStripMenuItem.Click += new System.EventHandler(this.hexagonToolStripMenuItem_Click);
            // 
            // mapViewPanel
            // 
            this.mapViewPanel.AutoSize = true;
            this.mapViewPanel.Controls.Add(this.mapViewPictureBox);
            this.mapViewPanel.Location = new System.Drawing.Point(0, 29);
            this.mapViewPanel.Name = "mapViewPanel";
            this.mapViewPanel.Size = new System.Drawing.Size(447, 262);
            this.mapViewPanel.TabIndex = 1;
            this.mapViewPanel.Paint += new System.Windows.Forms.PaintEventHandler(this.mapViewPanel_Paint);
            this.mapViewPanel.MouseClick += new System.Windows.Forms.MouseEventHandler(this.mapViewPanel_MouseClick);
            // 
            // mapViewPictureBox
            // 
            this.mapViewPictureBox.BackColor = System.Drawing.Color.Transparent;
            this.mapViewPictureBox.Location = new System.Drawing.Point(0, 0);
            this.mapViewPictureBox.Name = "mapViewPictureBox";
            this.mapViewPictureBox.Size = new System.Drawing.Size(444, 259);
            this.mapViewPictureBox.TabIndex = 0;
            this.mapViewPictureBox.TabStop = false;
            this.mapViewPictureBox.Paint += new System.Windows.Forms.PaintEventHandler(this.mapViewPictureBox_Paint);
            // 
            // mapViewOpenFileDialog
            // 
            this.mapViewOpenFileDialog.Filter = "XML Files (*.xml)|*.xml|All Files (*.*)|*.*";
            this.mapViewOpenFileDialog.FileOk += new System.ComponentModel.CancelEventHandler(this.mapViewOpenFileDialog_FileOk);
            // 
            // mapViewSaveFileDialog
            // 
            this.mapViewSaveFileDialog.FileOk += new System.ComponentModel.CancelEventHandler(this.mapViewSaveFileDialog_FileOk);
            // 
            // toolbarForm
            //  
            this.toolbarForm.ClientSize = new System.Drawing.Size(215, 385);
            this.toolbarForm.Location = new System.Drawing.Point(10, 10);
            this.toolbarForm.Name = "toolbarForm";
            this.toolbarForm.Text = "Toolbar";
            this.toolbarForm.Visible = false;
			// 
            // cmsAddPoint
            // 
            this.cmsAddPoint.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.startToolStripMenuItem,
            this.endToolStripMenuItem});
            this.cmsAddPoint.Name = "cmsAddPoint";
            this.cmsAddPoint.Size = new System.Drawing.Size(103, 48);
            // 
            // startToolStripMenuItem
            // 
            this.startToolStripMenuItem.Name = "startToolStripMenuItem";
            this.startToolStripMenuItem.Size = new System.Drawing.Size(102, 22);
            this.startToolStripMenuItem.Text = "start";
            this.startToolStripMenuItem.Click += new System.EventHandler(this.startToolStripMenuItem_Click);
            // 
            // endToolStripMenuItem
            // 
            this.endToolStripMenuItem.Name = "endToolStripMenuItem";
            this.endToolStripMenuItem.Size = new System.Drawing.Size(102, 22);
            this.endToolStripMenuItem.Text = "end";
            this.endToolStripMenuItem.Click += new System.EventHandler(this.endToolStripMenuItem_Click);
            // 
            // MapViewForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSize = true;
            this.ClientSize = new System.Drawing.Size(494, 299);
            this.Controls.Add(this.mapViewPanel);
            this.Controls.Add(this.mapViewMenuStrip);
            this.MainMenuStrip = this.mapViewMenuStrip;
            this.Name = "MapViewForm";
            this.Text = "MapViewForm";
            this.Paint += new System.Windows.Forms.PaintEventHandler(this.MapViewForm_Paint);
            this.mapViewMenuStrip.ResumeLayout(false);
            this.mapViewMenuStrip.PerformLayout();
            this.mapViewPanel.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.mapViewPictureBox)).EndInit();
            this.cmsAddPoint.ResumeLayout(false);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.MenuStrip mapViewMenuStrip;
        private System.Windows.Forms.ToolStripMenuItem fileToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem openToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem saveToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem exitToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem viewToolStripMenuItem;
        private System.Windows.Forms.Panel mapViewPanel;
        private System.Windows.Forms.PictureBox mapViewPictureBox;
        private System.Windows.Forms.ToolStripMenuItem toobarToolStripMenuItem;
        private System.Windows.Forms.OpenFileDialog mapViewOpenFileDialog;
        private System.Windows.Forms.SaveFileDialog mapViewSaveFileDialog;
        private System.Windows.Forms.ToolStripMenuItem mapViewToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem mapToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem hexagonToolStripMenuItem;
        private PathPlanner.ToolbarForm toolbarForm;
        private System.Windows.Forms.ToolStripMenuItem configToolStripMenuItem;
        private System.Windows.Forms.ContextMenuStrip cmsAddPoint;
        private System.Windows.Forms.ToolStripMenuItem startToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem endToolStripMenuItem;
    }
}