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
            this.SuspendLayout();
            // 
            // tbPlanLength
            // 
            this.tbPlanLength.Location = new System.Drawing.Point(70, 11);
            this.tbPlanLength.Name = "tbPlanLength";
            this.tbPlanLength.Size = new System.Drawing.Size(100, 21);
            this.tbPlanLength.TabIndex = 0;
            // 
            // lbPlanLength
            // 
            this.lbPlanLength.AutoSize = true;
            this.lbPlanLength.Location = new System.Drawing.Point(0, 11);
            this.lbPlanLength.Name = "lbPlanLength";
            this.lbPlanLength.Size = new System.Drawing.Size(71, 12);
            this.lbPlanLength.TabIndex = 1;
            this.lbPlanLength.Text = "Plan Length";
            // 
            // rbInfoMax
            // 
            this.rbInfoMax.AutoSize = true;
            this.rbInfoMax.Location = new System.Drawing.Point(11, 63);
            this.rbInfoMax.Name = "rbInfoMax";
            this.rbInfoMax.Size = new System.Drawing.Size(113, 16);
            this.rbInfoMax.TabIndex = 2;
            this.rbInfoMax.TabStop = true;
            this.rbInfoMax.Text = "Information Max";
            this.rbInfoMax.UseVisualStyleBackColor = true;
            this.rbInfoMax.Click += new System.EventHandler(this.rbInfoMax_Click);
            // 
            // rbRandom
            // 
            this.rbRandom.AutoSize = true;
            this.rbRandom.Location = new System.Drawing.Point(11, 85);
            this.rbRandom.Name = "rbRandom";
            this.rbRandom.Size = new System.Drawing.Size(59, 16);
            this.rbRandom.TabIndex = 3;
            this.rbRandom.TabStop = true;
            this.rbRandom.Text = "Random";
            this.rbRandom.UseVisualStyleBackColor = true;
            this.rbRandom.Click += new System.EventHandler(this.rbRandom_Click);
            // 
            // tbPlan
            // 
            this.tbPlan.Location = new System.Drawing.Point(11, 116);
            this.tbPlan.Name = "tbPlan";
            this.tbPlan.Size = new System.Drawing.Size(75, 21);
            this.tbPlan.TabIndex = 4;
            this.tbPlan.Text = "Plan Path";
            this.tbPlan.UseVisualStyleBackColor = true;
            this.tbPlan.Click += new System.EventHandler(this.tbPlan_Click);
            // 
            // rbDistMin
            // 
            this.rbDistMin.AutoSize = true;
            this.rbDistMin.Location = new System.Drawing.Point(11, 41);
            this.rbDistMin.Name = "rbDistMin";
            this.rbDistMin.Size = new System.Drawing.Size(95, 16);
            this.rbDistMin.TabIndex = 5;
            this.rbDistMin.TabStop = true;
            this.rbDistMin.Text = "Distance Min";
            this.rbDistMin.UseVisualStyleBackColor = true;
            this.rbDistMin.Click += new System.EventHandler(this.rbDistMin_Click);
            // 
            // ToolbarForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(199, 320);
            this.Controls.Add(this.rbDistMin);
            this.Controls.Add(this.tbPlan);
            this.Controls.Add(this.rbRandom);
            this.Controls.Add(this.rbInfoMax);
            this.Controls.Add(this.lbPlanLength);
            this.Controls.Add(this.tbPlanLength);
            this.Name = "ToolbarForm";
            this.Text = "Toobar";
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
    }
}