using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;

namespace SerialROVCS
{
    public partial class Form1 : Form
    {
        private SerialPort myport;
        public Form1()
        {
            InitializeComponent();
            init();
        }
        

        private void On_btn_Click(object sender, EventArgs e)
        {
            myport.WriteLine("T");

            On_btn.Enabled = false;
            Off_btn.Enabled = true;
        }
        private void init()
        {
            try
            {
                myport = new SerialPort();
                myport.BaudRate = 9600;
                myport.PortName = "COM5";
                myport.Open();
            }
            catch(Exception)
            {
                MessageBox.Show("Error: Cannot open port");
            }
            //
            On_btn.Enabled = true;
            Off_btn.Enabled = false;
        }
        private void Off_btn_Click(object sender, EventArgs e)
        {
            myport.WriteLine("F");

            On_btn.Enabled = true;
            Off_btn.Enabled = false;
        }
    }
}
