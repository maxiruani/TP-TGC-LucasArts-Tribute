using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using Microsoft.DirectX.Direct3D;
using TgcViewer;
using TgcViewer.Utils.Sound;

namespace AlumnoEjemplos.LucasArtsTribute.Sound
{
    class Mp3
    {
        /// <summary>
        /// Cargar un nuevo MP3 si hubo una variacion
        /// </summary>
        public  Mp3(string filePath)
        {
            GuiController.Instance.Mp3Player.closeFile();
            _currentPath = GuiController.Instance.AlumnoEjemplosMediaDir + filePath;
            GuiController.Instance.Mp3Player.FileName = _currentPath;
            _player = GuiController.Instance.Mp3Player;

        }

        private String _currentPath;
        private TgcMp3Player _player;

        public void Play()
        {
            _player.play(true);
        }

        public void Stop()
        {
            _player.stop();
            
        }

        public bool IsPlaying()
        {
            if (TgcMp3Player.States.Playing == _player.getStatus())
                return true;
            return false;
        }

    }
}
