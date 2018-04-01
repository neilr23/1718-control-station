using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace DllCTest
{
    public partial class Form1:Form
    {
        private int clicker;
        public Form1()
        {
            InitializeComponent();
            clicker = 0;
        }
        private void toolStripDropDownButton1_Click(object sender, EventArgs e)
        {

        }
        private void exitToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Application.Exit();
        }

        private void speedNameLabel_Click(object sender, EventArgs e)
        {

        }
    }
}
