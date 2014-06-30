namespace WingmanPathPlanning
{
    partial class SimTestForm
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
            this.mapDataGridView = new System.Windows.Forms.DataGridView();
            this.btnClear = new System.Windows.Forms.Button();
            this.btnApply = new System.Windows.Forms.Button();
            this.comboBoxMtdSel = new System.Windows.Forms.ComboBox();
            this.btnPlanStep = new System.Windows.Forms.Button();
            this.checkBoxDraw = new System.Windows.Forms.CheckBox();
            ((System.ComponentModel.ISupportInitialize)(this.mapDataGridView)).BeginInit();
            this.SuspendLayout();
            // 
            // mapDataGridView
            // 
            this.mapDataGridView.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.mapDataGridView.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            this.mapDataGridView.Location = new System.Drawing.Point(386, 0);
            this.mapDataGridView.Name = "mapDataGridView";
            this.mapDataGridView.RowHeadersWidth = 20;
            this.mapDataGridView.RowTemplate.Height = 23;
            this.mapDataGridView.Size = new System.Drawing.Size(306, 253);
            this.mapDataGridView.TabIndex = 0;
            this.mapDataGridView.CellEndEdit += new System.Windows.Forms.DataGridViewCellEventHandler(this.mapDataGridView_CellEndEdit);
            // 
            // btnClear
            // 
            this.btnClear.Location = new System.Drawing.Point(12, 274);
            this.btnClear.Name = "btnClear";
            this.btnClear.Size = new System.Drawing.Size(63, 26);
            this.btnClear.TabIndex = 1;
            this.btnClear.Text = "Clear";
            this.btnClear.UseVisualStyleBackColor = true;
            this.btnClear.Click += new System.EventHandler(this.btnClear_Click);
            // 
            // btnApply
            // 
            this.btnApply.Location = new System.Drawing.Point(81, 274);
            this.btnApply.Name = "btnApply";
            this.btnApply.Size = new System.Drawing.Size(63, 26);
            this.btnApply.TabIndex = 2;
            this.btnApply.Text = "Apply";
            this.btnApply.UseVisualStyleBackColor = true;
            this.btnApply.Click += new System.EventHandler(this.btnApply_Click);
            // 
            // comboBoxMtdSel
            // 
            this.comboBoxMtdSel.FormattingEnabled = true;
            this.comboBoxMtdSel.Location = new System.Drawing.Point(150, 278);
            this.comboBoxMtdSel.Name = "comboBoxMtdSel";
            this.comboBoxMtdSel.Size = new System.Drawing.Size(197, 20);
            this.comboBoxMtdSel.TabIndex = 3;
            this.comboBoxMtdSel.SelectedIndexChanged += new System.EventHandler(this.comboBoxMtdSel_SelectedIndexChanged);
            // 
            // btnPlanStep
            // 
            this.btnPlanStep.Location = new System.Drawing.Point(353, 276);
            this.btnPlanStep.Name = "btnPlanStep";
            this.btnPlanStep.Size = new System.Drawing.Size(63, 23);
            this.btnPlanStep.TabIndex = 4;
            this.btnPlanStep.Text = "Step";
            this.btnPlanStep.UseVisualStyleBackColor = true;
            // 
            // checkBoxDraw
            // 
            this.checkBoxDraw.AutoSize = true;
            this.checkBoxDraw.Location = new System.Drawing.Point(423, 278);
            this.checkBoxDraw.Name = "checkBoxDraw";
            this.checkBoxDraw.Size = new System.Drawing.Size(48, 16);
            this.checkBoxDraw.TabIndex = 5;
            this.checkBoxDraw.Text = "Draw";
            this.checkBoxDraw.UseVisualStyleBackColor = true;
            // 
            // SimTestForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(693, 302);
            this.Controls.Add(this.checkBoxDraw);
            this.Controls.Add(this.btnPlanStep);
            this.Controls.Add(this.comboBoxMtdSel);
            this.Controls.Add(this.btnApply);
            this.Controls.Add(this.btnClear);
            this.Controls.Add(this.mapDataGridView);
            this.Name = "SimTestForm";
            this.Text = "SimTestForm";
            this.Load += new System.EventHandler(this.SimTestForm_Load);
            this.Paint += new System.Windows.Forms.PaintEventHandler(this.SimTestForm_Paint);
            this.MouseClick += new System.Windows.Forms.MouseEventHandler(this.SimTestForm_MouseClick);
            ((System.ComponentModel.ISupportInitialize)(this.mapDataGridView)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.DataGridView mapDataGridView;
        private System.Windows.Forms.Button btnClear;
        private System.Windows.Forms.Button btnApply;
        private System.Windows.Forms.ComboBox comboBoxMtdSel;
        private System.Windows.Forms.Button btnPlanStep;
        private System.Windows.Forms.CheckBox checkBoxDraw;

    }
}