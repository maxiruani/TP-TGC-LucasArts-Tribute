using System;
using System.Collections.Generic;
using System.Drawing;
using System.Reflection;
using TgcViewer.Utils._2D;

namespace AlumnoEjemplos.LucasArtsTribute
{
    public class Instrumental
    {
        public Instrumental(CarStats stats)
        {
            _stats = stats;
            _valuesToShow = new Dictionary<string, float>();
            RefreshValues();
            CreateTextList();
        }

        public List<TgcText2d> GetValues()
        {
            RefreshValues();
            RefreshTexts();
            return _textToShow;
        }


        private void RefreshValues()
        {
            Type myType = _stats.GetType();
            IList<PropertyInfo> props = new List<PropertyInfo>(myType.GetProperties());
            foreach (PropertyInfo prop in props)
            {
                _valuesToShow[prop.Name] = (float)prop.GetValue(_stats, null);
            }
        }

        private void RefreshTexts()
        {
            foreach (KeyValuePair<string, float> pair in _valuesToShow)
            {
                TgcText2d text = _textToShow.Find(item => item.Text.Contains(pair.Key));
                if (text == null)
                    continue;
                text.Text = pair.Key + ": " + pair.Value;
            }
        }


        private void CreateTextList()
        {
            _textToShow = new List<TgcText2d>();
            TgcText2d text;
            int offsetYPosition = 0;
            int offsetYSize = 0;
            foreach (KeyValuePair<string, float> pair in _valuesToShow)
            {
                text = new TgcText2d();
                _textToShow.Add(text);
                text.Text = pair.Key + ": " + pair.Value;
                text.Color = Color.Yellow;
                text.changeFont(new Font("TimesNewRoman", 25, FontStyle.Bold | FontStyle.Italic));
                text.Position = new Point(XDefault, YDefault);
                text.Size = new Size(XDefault, YDefault);
                text.Position = new Point(XDefault, YDefault + offsetYPosition);
                text.Size = new Size(XDefault, YDefault + offsetYPosition);
                offsetYPosition += 25;
                offsetYSize += 25;
            }
                
            
        }

        private CarStats _stats;
        private List<TgcText2d> _textToShow; 
        private Dictionary<String, float> _valuesToShow;
        const int XDefault = 500;
        const int YDefault = 500;

    }
}
