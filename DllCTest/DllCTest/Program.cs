using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using System.Runtime.InteropServices;

namespace DllCTest
{
    class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]

        [DllImport("C:\\rov\\1718 - control - station\\DllTest")]
            public static extern IntPtr Initialize();
        [DllImport("C:\\rov\\1718 - control - station\\DllTest")]
            public static extern int readSerialPort();
        [DllImport("C:\\rov\\1718 - control - station\\DllTest")]
            public static extern bool writeSerialPort();
        [DllImport("C:\\rov\\1718 - control - station\\DllTest")]
            public static extern bool isConnected();
        static void Main(string[] args)
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new Form1());
        }
    }
}
