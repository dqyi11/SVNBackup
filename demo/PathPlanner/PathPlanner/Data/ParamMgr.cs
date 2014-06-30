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
                config.Save(ConfigurationSaveMode.Modified);

                ConfigurationManager.RefreshSection("appSettings");
            }
            else
            {
                this.hexagonalSize = Int32.Parse(ConfigurationManager.AppSettings["HexagonalSize"]);

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

            this.hexagonalSize = Int32.Parse(this.configForm.hexagonalSize);

            config.Save(ConfigurationSaveMode.Modified);
            ConfigurationManager.RefreshSection("appSettings");

        }
    }
}
