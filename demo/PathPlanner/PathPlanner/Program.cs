using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using System.IO;

namespace PathPlanner
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            string logFilename = "Log-" + DateTime.Now.Ticks + ".txt";
            StreamWriter sw = new StreamWriter(logFilename);
            Console.SetOut(sw);
            sw.AutoFlush = true;

            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new MapViewForm());

            sw.Flush();
            sw.Close();
        }
    }
}
