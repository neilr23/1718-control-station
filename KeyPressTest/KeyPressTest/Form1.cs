﻿using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace KeyPressTest
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }
        private void Form1_KeyPress(object sender, KeyPressEventArgs e)
        {
            charLabel.Text = "Key pressed: " + e.KeyChar;
        }
        private void Form1_KeyDown(object sender, KeyPressEventArgs e)
        {
            infoLabel.Text = 
                "Alt: "
                "Shift: "
                "Ctrl: "
               
        }
    }
}