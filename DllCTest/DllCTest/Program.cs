using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using System.Runtime.InteropServices;

namespace DllCTest
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]

        [DllImport("C:\\rov\\1718 - control - station\\DllTest")]
            public static extern void ComLoop();
            public static extern void initialize(string portname);
        static void Main(string[] args)
        {
            Assembly myAssembly;
            myAssembly = Assembly.LoadFile("DllTest.cpp");

            object o;
            Type myType = myAssembly.GetType("<assembly>.<class>");
            o = Activator.CreateInstance(myType);
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new Form1());
        }
    }
}
