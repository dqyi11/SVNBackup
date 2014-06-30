using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Configuration;

namespace PathPlanner.Data
{
    public class ParamMgr
    {
        private PathPlanner.ParamConfigForm configForm;

        public int hexagonalSize = 5;
        public int wingmanConstraint = 2;
        public int humanObs = 2;
        public int robotObs = 2;
        Configuration config;

        public bool IsView
        {
            get
            {
                return configForm.Visible;
            }
        }

        public ParamMgr()
        {
            configForm = new PathPlanner.ParamConfigForm(this);
            configForm.ClientSize = new System.Drawing.Size(470, 270);
            configForm.Location = new System.Drawing.Point(40, 40);
            configForm.Visible = false;

            string configFile = AppDomain.CurrentDomain.SetupInformation.ConfigurationFile;

            if (!System.IO.File.Exists(configFile))
            {
                Console.Write("Not init");
              
                config = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                AppSettingsSection app = config.AppSettings;
                app.Settings.Add("HexagonalSize", this.hexagonalSize.ToString());
                app.Settings.Add("WingmanConstraint", this.wingmanConstraint.ToString());
                app.Settings.Add("HumanObservation", this.humanObs.ToString());
                app.Settings.Add("RobotObservation", this.robotObs.ToString());
                config.Save(ConfigurationSaveMode.Modified);

                ConfigurationManager.RefreshSection("appSettings");
            }
            else
            {
                this.hexagonalSize = Int32.Parse(ConfigurationManager.AppSettings["HexagonalSize"]);
                this.wingmanConstraint = Int32.Parse(ConfigurationManager.AppSettings["WingmanConstraint"]);
                this.humanObs = Int32.Parse(ConfigurationManager.AppSettings["HumanObservation"]);
                this.robotObs = Int32.Parse(ConfigurationManager.AppSettings["RobotObservation"]);
            }

        }

        public void ViewForm(bool show)
        {
            configForm.Visible = show;
        }

        public void Save()
        {
            AppSettingsSection app = config.AppSettings;

            app.Settings["HexagonalSize"].Value = this.configForm.hexagonalSize;
            app.Settings["WingmanConstraint"].Value = this.configForm.wingmanConstraint;
            app.Settings["HumanObservation"].Value = this.configForm.humanObservation;
            app.Settings["RobotObservation"].Value = this.configForm.robotObservation;

            this.hexagonalSize = Int32.Parse(this.configForm.hexagonalSize);
            this.wingmanConstraint = Int32.Parse(this.configForm.wingmanConstraint);
            this.humanObs = Int32.Parse(this.configForm.humanObservation);
            this.robotObs = Int32.Parse(this.configForm.robotObservation);

            config.Save(ConfigurationSaveMode.Modified);
            ConfigurationManager.RefreshSection("appSettings");

        }
    }
}
