using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Globalization;

namespace AlumnoEjemplos.LucasArtsTribute.Utils
{
    public class ConfigurationManager
    {
        private Dictionary<String, String> dictionary;

        public Dictionary<String, String> Dictionary
        {
            get { return dictionary; }
            set { dictionary = value; }
        }

        private char splitValue;

        public char SplitValue
        {
            get { return splitValue; }
            set { splitValue = value; }
        }

        private NumberFormatInfo ni = null;

        public ConfigurationManager()
        {
            splitValue = ':';
            dictionary = new Dictionary<String, String>();

            CultureInfo ci = CultureInfo.InstalledUICulture;
            ni = (System.Globalization.NumberFormatInfo) ci.NumberFormat.Clone();
            ni.NumberDecimalSeparator = ".";
        }

        public bool ReadConfigFile(String path)
        {
            String line;

            try
            {
                StreamReader sr = new StreamReader(path);

                line = sr.ReadLine();

                while (line != null)
                {
                    if (!line.Contains(splitValue))
                        return false;

                    String[] values = line.Split(splitValue);

                    dictionary[values[0].Trim()] = values[1].Trim();

                    line = sr.ReadLine();
                }

                sr.Close();
            }
            catch (Exception e)
            {
                return false;
            }

            return true;
        }

        public String GetValue(String key)
        {
            return dictionary[key];
        }

        public float GetFloatValue(String key)
        {
            return float.Parse(dictionary[key], ni);
        }

        public int GetIntValue(String key)
        {
            return int.Parse(dictionary[key]);
        }

    }
}
