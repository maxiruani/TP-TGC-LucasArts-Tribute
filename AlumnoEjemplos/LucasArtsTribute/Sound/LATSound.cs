using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.DirectX.DirectSound;
using Microsoft.DirectX;
using TgcViewer;

namespace AlumnoEjemplos.LucasArtsTribute.Sound
{
    public class LATSound
    {
        private SecondaryBuffer soundBuffer;

        public LATSound(String path)
        {
            LoadSound(path);
        }

        private void LoadSound(String path)
        {
            try
            {
                Dispose();

                BufferDescription bufferDescription = new BufferDescription();
                bufferDescription.ControlVolume = true;
                bufferDescription.ControlFrequency = true;
                bufferDescription.ControlPan = true;

                soundBuffer = new SecondaryBuffer(GuiController.Instance.AlumnoEjemplosMediaDir + path, bufferDescription, GuiController.Instance.DirectSound.DsDevice);
            }
            catch (Exception e)
            {
                throw new Exception("Error al cargar sonido estático WAV: " + path, e);
            }
        }

        // Liberar los recursos del buffer
        public void Dispose()
        {
            if (soundBuffer != null && !soundBuffer.Disposed)
            {
                soundBuffer.Dispose();
                soundBuffer = null;
            }
        }

        // Manejo de posicion
        public int Position
        {
            get
            {
                int play, write;
                soundBuffer.GetCurrentPosition(out play, out write);
                return play;
            }
            set
            {
                soundBuffer.SetCurrentPosition(value);
            }
        }

        // Manejo de comienzo, fin, bucle de sonido
        public void Play()
        {
            Position = 0;
            soundBuffer.Play(0, BufferPlayFlags.Default);
        }

        public void Loop()
        {
            Position = 0;
            soundBuffer.Play(0, BufferPlayFlags.Looping);
        }

        public void SetLoopPosition(int pos)
        {
            if (!Playing) 
                Play(); 
            
            if (Position > pos) 
                Play();
        }

        public void Stop()
        {
            soundBuffer.Stop();
        }

        public bool Playing
        {
            get { return soundBuffer.Status.Playing; }
        }

        public bool Looping
        {
            get { return soundBuffer.Status.Looping; }
        }

        // Manejos varios
        public int Volume
        {
            get { return ((soundBuffer.Volume + 10000) / 100); }
            set { soundBuffer.Volume = (value * 100) - 10000; }
        }

        public int Pan
        {
            get { return soundBuffer.Pan; }
            set { soundBuffer.Pan = value; }
        }

        public int Frequency
        {
            get { return soundBuffer.Frequency; }
            set { soundBuffer.Frequency = value; }
        }

    }
}
