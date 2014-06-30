using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using PixelMap;
using MapLabeling.data;
using System.IO;
using System.Xml;

namespace MapLabeling
{
    public partial class MapViewerForm : Form
    {
        private PixelMap.PixelMap pixelMap;

        IndoorLabel tempIndoorLabel = null;
        OutdoorLabel tempOutdoorLabel = null;

        private MapInfoManager infoMgr;
        private string workingPath;
        private string mapFilepath;

        public enum FormState
        {
            NORMAL = 1,
            ADD_FEATURE = 2,
            ADD_INDOOR = 3,
            ADD_OUTDOOR = 4,
        };

        FormState formState;

        public MapViewerForm()
        {
            InitializeComponent();
            this.toolbarForm.parentForm = this;
            this.formState = FormState.NORMAL;

            infoMgr = new MapInfoManager();

            this.pixelMap = null;
        }

        private void openToolStripMenuItem_Click(object sender, EventArgs e)
        {
            this.viewerOpenFileDialog.ShowDialog();
        }

        private void viewerOpenFileDialog_FileOk(object sender, CancelEventArgs e)
        {
            try
            {
                this.pixelMap = new PixelMap.PixelMap(this.viewerOpenFileDialog.FileName);
                this.mapFilepath = this.viewerOpenFileDialog.FileName;
                this.viewerPictureBox.BackgroundImage = this.pixelMap.GreyMap;
                this.viewerPictureBox.Size = new Size(this.pixelMap.Header.Width, this.pixelMap.Header.Height);
                this.viewerPanel.AutoScroll = true;
                this.toolbarForm.Visible = true;

                this.infoMgr.mapFilename = System.IO.Path.GetFileName(this.viewerOpenFileDialog.FileName);
                this.infoMgr.mapWidth = this.pixelMap.Header.Width;
                this.infoMgr.mapHeight = this.pixelMap.Header.Height;
       
                this.saveToolStripMenuItem.Enabled = true;
            }
            catch (Exception ex)
            {
                ShowException(ex);
            }
        }

        private void ShowException(Exception ex)
        {
            string message = ex.InnerException.Message;
            string caption = "PixelMap Error! " + ex.Message;
            MessageBox.Show(message, caption, MessageBoxButtons.OK, MessageBoxIcon.Error);
        }

        private void toolbarToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (this.toolbarToolStripMenuItem.Checked == true)
            {
                this.toolbarForm.Visible = false;
                this.toolbarToolStripMenuItem.Checked = false;
            }
            else
            {
                this.toolbarForm.Visible = true;
                this.toolbarForm.BringToFront();
                this.toolbarToolStripMenuItem.Checked = true;
            }
        }

        public void SetFormState(FormState state)
        {
            formState = state;
        }

        private void viewerPictureBox_MouseEnter(object sender, EventArgs e)
        {
            if (formState == FormState.ADD_FEATURE)
            {
                this.Cursor = Cursors.Cross;
            }
        }

        private void viewerPictureBox_MouseClick(object sender, MouseEventArgs e)
        {
            if (formState == FormState.ADD_FEATURE)
            {
                FeatureLabel feature = this.infoMgr.featureMgr.CreateLabel();
                feature.pos = new Point(e.X, e.Y);
                FeaturePropertyForm propertyForm = new FeaturePropertyForm(feature, this.infoMgr.featureMgr);

                propertyForm.ShowDialog();
                Refresh();
            }
            else if (formState == FormState.ADD_INDOOR)
            {
                MouseEventArgs mouseEvent = (MouseEventArgs)e;
                if (mouseEvent.Button == MouseButtons.Right)
                {
                    // pop to choose "complete" or "cancel"
                    this.polygonContextMenuStrip.Show(this.Left +  e.X,this.Top + e.Y);
                }
                else if(mouseEvent.Button == MouseButtons.Left)
                {

                    if (this.tempIndoorLabel == null)
                    {
                        this.tempIndoorLabel = this.infoMgr.indoorMgr.CreateLabel();
                    }
                    this.tempIndoorLabel.AddVertice(new Point(e.X, e.Y));

                }

                Refresh();
            }
            else if (formState == FormState.ADD_OUTDOOR)
            {
                MouseEventArgs mouseEvent = (MouseEventArgs)e;
                if (mouseEvent.Button == MouseButtons.Right)
                {
                    // pop to choose "complete" or "cancel"
                    this.polygonContextMenuStrip.Show(this.Left + e.X, this.Top + e.Y);
                }
                else if (mouseEvent.Button == MouseButtons.Left)
                {

                    if (this.tempOutdoorLabel == null)
                    {
                        this.tempOutdoorLabel = this.infoMgr.outdoorMgr.CreateLabel();
                    }
                    this.tempOutdoorLabel.AddVertice(new Point(e.X, e.Y));

                }

                Refresh();
            }
            else
            {
                MouseEventArgs mouseEvent = (MouseEventArgs)e;
                if (mouseEvent.Button == MouseButtons.Right)
                {
                    this.editContextMenuStrip.Show(this.Left + e.X, this.Top + e.Y);
                }
                Refresh();
            }
        }

        private void viewerPictureBox_Paint(object sender, PaintEventArgs e)
        {
            this.infoMgr.outdoorMgr.Draw(e.Graphics);
            this.infoMgr.indoorMgr.Draw(e.Graphics);
            this.infoMgr.featureMgr.Draw(e.Graphics);

            if (this.tempIndoorLabel != null)
            {
                this.tempIndoorLabel.Draw(e.Graphics);
            }

            if (this.tempOutdoorLabel != null)
            {
                this.tempOutdoorLabel.Draw(e.Graphics);
            }
        }


        private void completeToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (formState == FormState.ADD_OUTDOOR)
            {
                if (this.tempOutdoorLabel != null)
                {
                    this.tempOutdoorLabel.Complete();
                    OutdoorPropertyForm propertyForm = new OutdoorPropertyForm(this.tempOutdoorLabel, this.infoMgr.outdoorMgr);
                    propertyForm.ShowDialog();

                    //this.outdoors.Add(this.tempOutdoorLabel);
                    this.tempOutdoorLabel = null;
                }
            }
            else if (formState == FormState.ADD_INDOOR)
            {
                if (this.tempIndoorLabel != null)
                {
                    this.tempIndoorLabel.Complete();
                    IndoorPropertyForm propertyForm = new IndoorPropertyForm(this.tempIndoorLabel, this.infoMgr.indoorMgr);
                    propertyForm.ShowDialog();

                    //this.indoors.Add(this.tempIndoorLabel);
                    this.tempIndoorLabel = null;
                }
            }

            Refresh();
        }

        private void cancelToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (this.tempIndoorLabel != null)
            {
                this.tempIndoorLabel.Clear();
            }

            if (this.tempOutdoorLabel != null)
            {
                this.tempOutdoorLabel.Clear();
            }

            Refresh();
        }

        private void saveToolStripMenuItem_Click(object sender, EventArgs e)
        {
            this.viewerSaveFileDialog.ShowDialog();
        }

        private void viewrSaveFileDialog_FileOk(object sender, CancelEventArgs e)
        {
            this.infoMgr.DumpToFile(this.viewerSaveFileDialog.FileName);
        }

        private void MapViewerForm_Paint(object sender, PaintEventArgs e)
        {
            if (this.toolbarForm.Visible == true)
            {
                this.toolbarToolStripMenuItem.Checked = true;
            }
            else
            {
                this.toolbarToolStripMenuItem.Checked = false;
            }
        }

        private void loadToolStripMenuItem_Click(object sender, EventArgs e)
        {
            this.viewerLoadFileDialog.ShowDialog();
        }

        private void viewerLoadFileDialog_FileOk(object sender, CancelEventArgs e)
        {
            try
            {
                workingPath = System.IO.Path.GetDirectoryName(this.viewerLoadFileDialog.FileName);
                infoMgr.LoadFile(this.viewerLoadFileDialog.FileName);

                this.mapFilepath = System.IO.Path.Combine(workingPath, infoMgr.mapFilename);

                this.pixelMap = new PixelMap.PixelMap(this.mapFilepath);
                this.viewerPictureBox.BackgroundImage = this.pixelMap.GreyMap;
                this.viewerPictureBox.Size = new Size(this.pixelMap.Header.Width, this.pixelMap.Header.Height);
                this.viewerPanel.AutoScroll = true;
                this.toolbarForm.Visible = true;

                this.saveToolStripMenuItem.Enabled = true;
            }
            catch (Exception ex)
            {
                ShowException(ex);
            }
        }

        private void exitToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Application.Exit();
        }

        private void worldFileToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (this.pixelMap != null)
            {
                this.openWorldFileDialog.ShowDialog();
            }
        }

        private void openWorldFileDialog_FileOk(object sender, CancelEventArgs e)
        {
            this.infoMgr.worldFilename = System.IO.Path.GetFileName(this.openWorldFileDialog.FileName);
        }

        private void viewerPictureBox_MouseMove(object sender, MouseEventArgs e)
        {
            if (this.formState == FormState.NORMAL)
            {
                int pos_x = e.X;
                int pos_y = e.Y;

                this.infoMgr.activeX = pos_x;
                this.infoMgr.activeY = pos_y;
                this.infoMgr.UpdateActiveLabel(pos_x, pos_y);
                Refresh();
            }
        }

        private void toolStripMenuItem1_Click(object sender, EventArgs e)
        {
            this.infoMgr.DeleteActiveLabel();
            Refresh();
        }
    }
}
