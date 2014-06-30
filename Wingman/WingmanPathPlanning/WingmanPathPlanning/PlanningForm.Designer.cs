namespace WingmanPathPlanning
{
    partial class PlanningForm
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
            this.cbMethodSelect = new System.Windows.Forms.ComboBox();
            this.lbMethod = new System.Windows.Forms.Label();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.btnPrint = new System.Windows.Forms.Button();
            this.txtBoxInfo = new System.Windows.Forms.TextBox();
            this.btnApply = new System.Windows.Forms.Button();
            this.btnStart = new System.Windows.Forms.Button();
            this.progressBar1 = new System.Windows.Forms.ProgressBar();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            // 
            // cbMethodSelect
            // 
            this.cbMethodSelect.FormattingEnabled = true;
            this.cbMethodSelect.Location = new System.Drawing.Point(65, 12);
            this.cbMethodSelect.Name = "cbMethodSelect";
            this.cbMethodSelect.Size = new System.Drawing.Size(333, 20);
            this.cbMethodSelect.TabIndex = 0;
            this.cbMethodSelect.SelectedIndexChanged += new System.EventHandler(this.cbMethodSelect_SelectedIndexChanged);
            // 
            // lbMethod
            // 
            this.lbMethod.AutoSize = true;
            this.lbMethod.Location = new System.Drawing.Point(12, 12);
            this.lbMethod.Name = "lbMethod";
            this.lbMethod.Size = new System.Drawing.Size(47, 12);
            this.lbMethod.TabIndex = 1;
            this.lbMethod.Text = "Method:";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.btnPrint);
            this.groupBox1.Controls.Add(this.txtBoxInfo);
            this.groupBox1.Controls.Add(this.btnApply);
            this.groupBox1.Controls.Add(this.btnStart);
            this.groupBox1.Location = new System.Drawing.Point(13, 38);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(502, 251);
            this.groupBox1.TabIndex = 2;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Info:";
            // 
            // btnPrint
            // 
            this.btnPrint.Location = new System.Drawing.Point(183, 222);
            this.btnPrint.Name = "btnPrint";
            this.btnPrint.Size = new System.Drawing.Size(119, 23);
            this.btnPrint.TabIndex = 6;
            this.btnPrint.Text = "Print";
            this.btnPrint.UseVisualStyleBackColor = true;
            // 
            // txtBoxInfo
            // 
            this.txtBoxInfo.Location = new System.Drawing.Point(7, 13);
            this.txtBoxInfo.Multiline = true;
            this.txtBoxInfo.Name = "txtBoxInfo";
            this.txtBoxInfo.Size = new System.Drawing.Size(489, 202);
            this.txtBoxInfo.TabIndex = 5;
            // 
            // btnApply
            // 
            this.btnApply.Location = new System.Drawing.Point(373, 221);
            this.btnApply.Name = "btnApply";
            this.btnApply.Size = new System.Drawing.Size(123, 23);
            this.btnApply.TabIndex = 4;
            this.btnApply.Text = "Apply";
            this.btnApply.UseVisualStyleBackColor = true;
            // 
            // btnStart
            // 
            this.btnStart.Location = new System.Drawing.Point(6, 222);
            this.btnStart.Name = "btnStart";
            this.btnStart.Size = new System.Drawing.Size(123, 23);
            this.btnStart.TabIndex = 3;
            this.btnStart.Text = "Start";
            this.btnStart.UseVisualStyleBackColor = true;
            this.btnStart.Click += new System.EventHandler(this.btnStart_Click);
            // 
            // progressBar1
            // 
            this.progressBar1.Location = new System.Drawing.Point(415, 12);
            this.progressBar1.Name = "progressBar1";
            this.progressBar1.Size = new System.Drawing.Size(100, 23);
            this.progressBar1.TabIndex = 3;
            // 
            // PlanningForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(527, 294);
            this.Controls.Add(this.progressBar1);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.lbMethod);
            this.Controls.Add(this.cbMethodSelect);
            this.Name = "PlanningForm";
            this.Text = "PlanningForm";
            this.Load += new System.EventHandler(this.PlanningForm_Load);
            this.Paint += new System.Windows.Forms.PaintEventHandler(this.PlanningForm_Paint);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ComboBox cbMethodSelect;
        private System.Windows.Forms.Label lbMethod;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Button btnStart;
        private System.Windows.Forms.Button btnApply;
        private System.Windows.Forms.TextBox txtBoxInfo;
        private System.Windows.Forms.ProgressBar progressBar1;
        private System.Windows.Forms.Button btnPrint;
    }
}