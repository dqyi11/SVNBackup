namespace MapLabeling
{
    partial class MapViewerForm
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
            this.viewerOpenFileDialog = new System.Windows.Forms.OpenFileDialog();
            this.viewerSaveFileDialog = new System.Windows.Forms.SaveFileDialog();
            this.fileToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.openToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.loadToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.saveToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.exitToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.viewerMenuStrip = new System.Windows.Forms.MenuStrip();
            this.viewToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.toolbarToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.importToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.worldFileToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.viewerPanel = new System.Windows.Forms.Panel();
            this.viewerPictureBox = new System.Windows.Forms.PictureBox();
            this.polygonContextMenuStrip = new System.Windows.Forms.ContextMenuStrip(this.components);
            this.completeToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.cancelToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.viewerLoadFileDialog = new System.Windows.Forms.OpenFileDialog();
            this.openWorldFileDialog = new System.Windows.Forms.OpenFileDialog();
            this.toolbarForm = new MapLabeling.ToolbarForm();
            this.editContextMenuStrip = new System.Windows.Forms.ContextMenuStrip(this.components);
            this.toolStripMenuItem1 = new System.Windows.Forms.ToolStripMenuItem();
            this.viewerMenuStrip.SuspendLayout();
            this.viewerPanel.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.viewerPictureBox)).BeginInit();
            this.polygonContextMenuStrip.SuspendLayout();
            this.editContextMenuStrip.SuspendLayout();
            this.SuspendLayout();
            // 
            // viewerOpenFileDialog
            // 
            this.viewerOpenFileDialog.Filter = "All files|*.*|Pixel Grey Map files|*.pgm";
            this.viewerOpenFileDialog.FileOk += new System.ComponentModel.CancelEventHandler(this.viewerOpenFileDialog_FileOk);
            // 
            // viewerSaveFileDialog
            // 
            this.viewerSaveFileDialog.Filter = "All files|*.*|XML files|*.xml";
            this.viewerSaveFileDialog.FileOk += new System.ComponentModel.CancelEventHandler(this.viewrSaveFileDialog_FileOk);
            // 
            // fileToolStripMenuItem
            // 
            this.fileToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.openToolStripMenuItem,
            this.loadToolStripMenuItem,
            this.saveToolStripMenuItem,
            this.exitToolStripMenuItem});
            this.fileToolStripMenuItem.Name = "fileToolStripMenuItem";
            this.fileToolStripMenuItem.Size = new System.Drawing.Size(37, 20);
            this.fileToolStripMenuItem.Text = "File";
            // 
            // openToolStripMenuItem
            // 
            this.openToolStripMenuItem.Name = "openToolStripMenuItem";
            this.openToolStripMenuItem.Size = new System.Drawing.Size(103, 22);
            this.openToolStripMenuItem.Text = "Open";
            this.openToolStripMenuItem.Click += new System.EventHandler(this.openToolStripMenuItem_Click);
            // 
            // loadToolStripMenuItem
            // 
            this.loadToolStripMenuItem.Name = "loadToolStripMenuItem";
            this.loadToolStripMenuItem.Size = new System.Drawing.Size(103, 22);
            this.loadToolStripMenuItem.Text = "Load";
            this.loadToolStripMenuItem.Click += new System.EventHandler(this.loadToolStripMenuItem_Click);
            // 
            // saveToolStripMenuItem
            // 
            this.saveToolStripMenuItem.Name = "saveToolStripMenuItem";
            this.saveToolStripMenuItem.Size = new System.Drawing.Size(103, 22);
            this.saveToolStripMenuItem.Text = "Save";
            this.saveToolStripMenuItem.Click += new System.EventHandler(this.saveToolStripMenuItem_Click);
            // 
            // exitToolStripMenuItem
            // 
            this.exitToolStripMenuItem.Name = "exitToolStripMenuItem";
            this.exitToolStripMenuItem.Size = new System.Drawing.Size(103, 22);
            this.exitToolStripMenuItem.Text = "Exit";
            this.exitToolStripMenuItem.Click += new System.EventHandler(this.exitToolStripMenuItem_Click);
            // 
            // viewerMenuStrip
            // 
            this.viewerMenuStrip.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.fileToolStripMenuItem,
            this.viewToolStripMenuItem,
            this.importToolStripMenuItem});
            this.viewerMenuStrip.Location = new System.Drawing.Point(0, 0);
            this.viewerMenuStrip.Name = "viewerMenuStrip";
            this.viewerMenuStrip.Size = new System.Drawing.Size(601, 24);
            this.viewerMenuStrip.TabIndex = 0;
            this.viewerMenuStrip.Text = "menuStrip1";
            // 
            // viewToolStripMenuItem
            // 
            this.viewToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.toolbarToolStripMenuItem});
            this.viewToolStripMenuItem.Name = "viewToolStripMenuItem";
            this.viewToolStripMenuItem.Size = new System.Drawing.Size(44, 20);
            this.viewToolStripMenuItem.Text = "View";
            // 
            // toolbarToolStripMenuItem
            // 
            this.toolbarToolStripMenuItem.Name = "toolbarToolStripMenuItem";
            this.toolbarToolStripMenuItem.Size = new System.Drawing.Size(115, 22);
            this.toolbarToolStripMenuItem.Text = "Toolbar";
            this.toolbarToolStripMenuItem.Click += new System.EventHandler(this.toolbarToolStripMenuItem_Click);
            // 
            // importToolStripMenuItem
            // 
            this.importToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.worldFileToolStripMenuItem});
            this.importToolStripMenuItem.Name = "importToolStripMenuItem";
            this.importToolStripMenuItem.Size = new System.Drawing.Size(55, 20);
            this.importToolStripMenuItem.Text = "Import";
            // 
            // worldFileToolStripMenuItem
            // 
            this.worldFileToolStripMenuItem.Name = "worldFileToolStripMenuItem";
            this.worldFileToolStripMenuItem.Size = new System.Drawing.Size(127, 22);
            this.worldFileToolStripMenuItem.Text = "World File";
            this.worldFileToolStripMenuItem.Click += new System.EventHandler(this.worldFileToolStripMenuItem_Click);
            // 
            // viewerPanel
            // 
            this.viewerPanel.AutoScroll = true;
            this.viewerPanel.AutoSize = true;
            this.viewerPanel.Controls.Add(this.viewerPictureBox);
            this.viewerPanel.Dock = System.Windows.Forms.DockStyle.Fill;
            this.viewerPanel.Location = new System.Drawing.Point(0, 24);
            this.viewerPanel.Name = "viewerPanel";
            this.viewerPanel.Size = new System.Drawing.Size(601, 399);
            this.viewerPanel.TabIndex = 2;
            // 
            // viewerPictureBox
            // 
            this.viewerPictureBox.Location = new System.Drawing.Point(0, 0);
            this.viewerPictureBox.Name = "viewerPictureBox";
            this.viewerPictureBox.Size = new System.Drawing.Size(601, 361);
            this.viewerPictureBox.SizeMode = System.Windows.Forms.PictureBoxSizeMode.AutoSize;
            this.viewerPictureBox.TabIndex = 0;
            this.viewerPictureBox.TabStop = false;
            this.viewerPictureBox.Paint += new System.Windows.Forms.PaintEventHandler(this.viewerPictureBox_Paint);
            this.viewerPictureBox.MouseClick += new System.Windows.Forms.MouseEventHandler(this.viewerPictureBox_MouseClick);
            this.viewerPictureBox.MouseEnter += new System.EventHandler(this.viewerPictureBox_MouseEnter);
            this.viewerPictureBox.MouseMove += new System.Windows.Forms.MouseEventHandler(this.viewerPictureBox_MouseMove);
            // 
            // polygonContextMenuStrip
            // 
            this.polygonContextMenuStrip.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.completeToolStripMenuItem,
            this.cancelToolStripMenuItem});
            this.polygonContextMenuStrip.Name = "polygonContextMenuStrip";
            this.polygonContextMenuStrip.Size = new System.Drawing.Size(127, 48);
            // 
            // completeToolStripMenuItem
            // 
            this.completeToolStripMenuItem.Name = "completeToolStripMenuItem";
            this.completeToolStripMenuItem.Size = new System.Drawing.Size(126, 22);
            this.completeToolStripMenuItem.Text = "Complete";
            this.completeToolStripMenuItem.Click += new System.EventHandler(this.completeToolStripMenuItem_Click);
            // 
            // cancelToolStripMenuItem
            // 
            this.cancelToolStripMenuItem.Name = "cancelToolStripMenuItem";
            this.cancelToolStripMenuItem.Size = new System.Drawing.Size(126, 22);
            this.cancelToolStripMenuItem.Text = "Cancel";
            this.cancelToolStripMenuItem.Click += new System.EventHandler(this.cancelToolStripMenuItem_Click);
            // 
            // viewerLoadFileDialog
            // 
            this.viewerLoadFileDialog.Filter = "All files|*.*|XML files|*.xml";
            this.viewerLoadFileDialog.FileOk += new System.ComponentModel.CancelEventHandler(this.viewerLoadFileDialog_FileOk);
            // 
            // openWorldFileDialog
            // 
            this.openWorldFileDialog.FileOk += new System.ComponentModel.CancelEventHandler(this.openWorldFileDialog_FileOk);
            // 
            // toolbarForm
            // 
            this.toolbarForm.ClientSize = new System.Drawing.Size(116, 200);
            this.toolbarForm.Location = new System.Drawing.Point(0, 0);
            this.toolbarForm.Name = "toolbarForm";
            this.toolbarForm.Text = "Toolbar";
            this.toolbarForm.Visible = false;
            // 
            // editContextMenuStrip
            // 
            this.editContextMenuStrip.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.toolStripMenuItem1});
            this.editContextMenuStrip.Name = "polygonContextMenuStrip";
            this.editContextMenuStrip.Size = new System.Drawing.Size(153, 48);
            // 
            // toolStripMenuItem1
            // 
            this.toolStripMenuItem1.Name = "toolStripMenuItem1";
            this.toolStripMenuItem1.Size = new System.Drawing.Size(152, 22);
            this.toolStripMenuItem1.Text = "Delete";
            this.toolStripMenuItem1.Click += new System.EventHandler(this.toolStripMenuItem1_Click);
            // 
            // MapViewerForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSize = true;
            this.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
            this.ClientSize = new System.Drawing.Size(601, 423);
            this.Controls.Add(this.viewerPanel);
            this.Controls.Add(this.viewerMenuStrip);
            this.MainMenuStrip = this.viewerMenuStrip;
            this.Name = "MapViewerForm";
            this.Text = "Map View";
            this.Paint += new System.Windows.Forms.PaintEventHandler(this.MapViewerForm_Paint);
            this.viewerMenuStrip.ResumeLayout(false);
            this.viewerMenuStrip.PerformLayout();
            this.viewerPanel.ResumeLayout(false);
            this.viewerPanel.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.viewerPictureBox)).EndInit();
            this.polygonContextMenuStrip.ResumeLayout(false);
            this.editContextMenuStrip.ResumeLayout(false);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.OpenFileDialog viewerOpenFileDialog;
        private System.Windows.Forms.SaveFileDialog viewerSaveFileDialog;
        private System.Windows.Forms.ToolStripMenuItem fileToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem openToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem saveToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem exitToolStripMenuItem;
        private System.Windows.Forms.MenuStrip viewerMenuStrip;
        private System.Windows.Forms.ToolStripMenuItem viewToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem toolbarToolStripMenuItem;
        private ToolbarForm toolbarForm;
        private System.Windows.Forms.Panel viewerPanel;
        private System.Windows.Forms.PictureBox viewerPictureBox;
        private System.Windows.Forms.ContextMenuStrip polygonContextMenuStrip;
        private System.Windows.Forms.ToolStripMenuItem completeToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem cancelToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem loadToolStripMenuItem;
        private System.Windows.Forms.OpenFileDialog viewerLoadFileDialog;
        private System.Windows.Forms.ToolStripMenuItem importToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem worldFileToolStripMenuItem;
        private System.Windows.Forms.OpenFileDialog openWorldFileDialog;
        private System.Windows.Forms.ContextMenuStrip editContextMenuStrip;
        private System.Windows.Forms.ToolStripMenuItem toolStripMenuItem1;
    }
}

