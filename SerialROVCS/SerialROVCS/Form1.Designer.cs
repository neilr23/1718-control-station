namespace SerialROVCS
{
    partial class Form1
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
            this.Off_btn = new System.Windows.Forms.Button();
            this.On_btn = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // Off_btn
            // 
            this.Off_btn.BackColor = System.Drawing.Color.Red;
            this.Off_btn.Location = new System.Drawing.Point(222, 86);
            this.Off_btn.Name = "Off_btn";
            this.Off_btn.Size = new System.Drawing.Size(153, 87);
            this.Off_btn.TabIndex = 0;
            this.Off_btn.Text = "OFF";
            this.Off_btn.UseVisualStyleBackColor = false;
            this.Off_btn.Click += new System.EventHandler(this.Off_btn_Click);
            // 
            // On_btn
            // 
            this.On_btn.BackColor = System.Drawing.Color.YellowGreen;
            this.On_btn.Location = new System.Drawing.Point(43, 86);
            this.On_btn.Name = "On_btn";
            this.On_btn.Size = new System.Drawing.Size(151, 87);
            this.On_btn.TabIndex = 1;
            this.On_btn.Text = "ON";
            this.On_btn.UseVisualStyleBackColor = false;
            this.On_btn.Click += new System.EventHandler(this.On_btn_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(683, 291);
            this.Controls.Add(this.On_btn);
            this.Controls.Add(this.Off_btn);
            this.Name = "Form1";
            this.Text = "Form1";
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Button Off_btn;
        private System.Windows.Forms.Button On_btn;
    }
}

