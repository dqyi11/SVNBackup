namespace MapLabeling
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
            this.checkboxPoint = new System.Windows.Forms.CheckBox();
            this.checkboxIndoor = new System.Windows.Forms.CheckBox();
            this.checkboxOutdoor = new System.Windows.Forms.CheckBox();
            this.checkboxEnemy = new System.Windows.Forms.CheckBox();
            this.SuspendLayout();
            // 
            // checkboxPoint
            // 
            this.checkboxPoint.Appearance = System.Windows.Forms.Appearance.Button;
            this.checkboxPoint.AutoSize = true;
            this.checkboxPoint.Location = new System.Drawing.Point(25, 12);
            this.checkboxPoint.Name = "checkboxPoint";
            this.checkboxPoint.Size = new System.Drawing.Size(53, 23);
            this.checkboxPoint.TabIndex = 0;
            this.checkboxPoint.Text = "Feature";
            this.checkboxPoint.UseVisualStyleBackColor = true;
            this.checkboxPoint.CheckedChanged += new System.EventHandler(this.Point_CheckedChanged);
            // 
            // checkboxIndoor
            // 
            this.checkboxIndoor.Appearance = System.Windows.Forms.Appearance.Button;
            this.checkboxIndoor.AutoSize = true;
            this.checkboxIndoor.Location = new System.Drawing.Point(25, 56);
            this.checkboxIndoor.Name = "checkboxIndoor";
            this.checkboxIndoor.Size = new System.Drawing.Size(47, 23);
            this.checkboxIndoor.TabIndex = 1;
            this.checkboxIndoor.Text = "Indoor";
            this.checkboxIndoor.UseVisualStyleBackColor = true;
            this.checkboxIndoor.CheckedChanged += new System.EventHandler(this.checkboxPolygon_CheckedChanged);
            // 
            // checkboxOutdoor
            // 
            this.checkboxOutdoor.Appearance = System.Windows.Forms.Appearance.Button;
            this.checkboxOutdoor.AutoSize = true;
            this.checkboxOutdoor.Location = new System.Drawing.Point(25, 99);
            this.checkboxOutdoor.Name = "checkboxOutdoor";
            this.checkboxOutdoor.Size = new System.Drawing.Size(55, 23);
            this.checkboxOutdoor.TabIndex = 3;
            this.checkboxOutdoor.Text = "Outdoor";
            this.checkboxOutdoor.UseVisualStyleBackColor = true;
            this.checkboxOutdoor.CheckedChanged += new System.EventHandler(this.checkBoxOutdoor_CheckedChanged);
            // 
            // checkboxEnemy
            // 
            this.checkboxEnemy.Appearance = System.Windows.Forms.Appearance.Button;
            this.checkboxEnemy.AutoSize = true;
            this.checkboxEnemy.Location = new System.Drawing.Point(27, 151);
            this.checkboxEnemy.Name = "checkboxEnemy";
            this.checkboxEnemy.Size = new System.Drawing.Size(49, 23);
            this.checkboxEnemy.TabIndex = 4;
            this.checkboxEnemy.Text = "Enemy";
            this.checkboxEnemy.UseVisualStyleBackColor = true;
            this.checkboxEnemy.CheckedChanged += new System.EventHandler(this.checkboxEnemy_CheckedChanged);
            // 
            // ToolbarForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(130, 268);
            this.Controls.Add(this.checkboxEnemy);
            this.Controls.Add(this.checkboxOutdoor);
            this.Controls.Add(this.checkboxIndoor);
            this.Controls.Add(this.checkboxPoint);
            this.Name = "ToolbarForm";
            this.Text = "Toolbar";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.CheckBox checkboxPoint;
        private System.Windows.Forms.CheckBox checkboxIndoor;
        private System.Windows.Forms.CheckBox checkboxOutdoor;
        private System.Windows.Forms.CheckBox checkboxEnemy;
    }
}