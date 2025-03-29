namespace DelaunayTriangulationTestCase1
{
	partial class Form1
	{
		/// <summary>
		///  Required designer variable.
		/// </summary>
		private System.ComponentModel.IContainer components = null;

		/// <summary>
		///  Clean up any resources being used.
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
		///  Required method for Designer support - do not modify
		///  the contents of this method with the code editor.
		/// </summary>
		private void InitializeComponent()
		{
			button1 = new Button();
			button2 = new Button();
			button3 = new Button();
			label1 = new Label();
			openFileDialog1 = new OpenFileDialog();
			button4 = new Button();
			SuspendLayout();
			// 
			// button1
			// 
			button1.Location = new Point(638, 70);
			button1.Name = "button1";
			button1.Size = new Size(123, 29);
			button1.TabIndex = 0;
			button1.Text = "Run";
			button1.UseVisualStyleBackColor = true;
			button1.Click += ClickRunButton;
			// 
			// button2
			// 
			button2.Location = new Point(638, 175);
			button2.Name = "button2";
			button2.Size = new Size(123, 29);
			button2.TabIndex = 1;
			button2.Text = "Save";
			button2.UseVisualStyleBackColor = true;
			button2.Click += ClickSaveButton;
			// 
			// button3
			// 
			button3.Location = new Point(638, 140);
			button3.Name = "button3";
			button3.Size = new Size(123, 29);
			button3.TabIndex = 2;
			button3.Text = "Load and Run";
			button3.UseVisualStyleBackColor = true;
			button3.Click += ClickLoadRunButton;
			// 
			// label1
			// 
			label1.AutoSize = true;
			label1.Location = new Point(587, 9);
			label1.Name = "label1";
			label1.Size = new Size(50, 20);
			label1.TabIndex = 3;
			label1.Text = "label1";
			// 
			// openFileDialog1
			// 
			openFileDialog1.FileName = "openFileDialog1";
			// 
			// button4
			// 
			button4.Location = new Point(638, 105);
			button4.Name = "button4";
			button4.Size = new Size(123, 29);
			button4.TabIndex = 4;
			button4.Text = "Rerun Same";
			button4.UseVisualStyleBackColor = true;
			button4.Click += ClickRerunButton;
			// 
			// Form1
			// 
			AutoScaleDimensions = new SizeF(8F, 20F);
			AutoScaleMode = AutoScaleMode.Font;
			ClientSize = new Size(800, 648);
			Controls.Add(button4);
			Controls.Add(label1);
			Controls.Add(button3);
			Controls.Add(button2);
			Controls.Add(button1);
			Name = "Form1";
			Text = "Form1";
			ResumeLayout(false);
			PerformLayout();
		}

		#endregion

		private Button button1;
		private Button button2;
		private Button button3;
		private Label label1;
		private OpenFileDialog openFileDialog1;
		private Button button4;
	}
}
