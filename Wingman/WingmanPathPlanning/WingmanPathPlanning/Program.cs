using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using WingmanPathPlanning.Hexagonal;
using WingmanPathPlanning.Map;
using WingmanPathPlanning.Data;
using System.Drawing;
using System.IO;

namespace WingmanPathPlanning
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


            ParameterManager mgr = new ParameterManager();
            mgr.Load();

            HexagonalMap map = new HexagonalMap(mgr.hexagonalWidthNum, mgr.hexagonalHeightNum, mgr.hexagonalSize, Hexagonal.HexOrientation.Pointy);
            map.GetState().BackgroundColor = Color.White;
            map.GetState().GridPenWidth = 2;
            map.GetState().ActiveHexBorderColor = Color.Red;
            map.GetState().ActiveHexBorderWidth = 2;        

            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);

            MapForm form = new MapForm(map, mgr);
            //form.Width = mgr.winFormWidth;
            //form.Height = mgr.winFormHeight;

            Application.Run(form);

            //Generate human path here
            
            

            sw.Flush();
            sw.Close();
        }
    }
}
