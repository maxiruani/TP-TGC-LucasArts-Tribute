using System;
using System.Collections.Generic;
using AlumnoEjemplos.LucasArtsTribute.Circuit;
using AlumnoEjemplos.LucasArtsTribute.Players;
using AlumnoEjemplos.LucasArtsTribute.Sound;
using AlumnoEjemplos.LucasArtsTribute.VehicleModel;
using Microsoft.DirectX;
using Microsoft.DirectX.DirectInput;
using TgcViewer;
using TgcViewer.Utils.Input;
using TgcViewer.Utils.TgcGeometry;
using TgcViewer.Utils.TgcSceneLoader;

namespace AlumnoEjemplos.LucasArtsTribute
{
    public class Player
    {
        public Player(TgcBox scenario, String carConfiguration, Vector3 initialPosition, int numberOfPlayer)
        {
            _playerNumber = numberOfPlayer;
            Car = new Vehicle(carConfiguration, initialPosition, Loader, UserControlsFactory.Create(numberOfPlayer));
            Piso = scenario;
            Cam = new Camara(initialPosition);
            Cam.SetCenterTargetUp(_camDelta, initialPosition, new Vector3(0, 100, 0), true); 
            Cam.Enable = true;
            LoadCamara(true);
            _nosRecolectedSound = new LATSound("LucasArtsTribute\\NosBottleSound.wav");
            NosCount = 0;
            //Reflejo en el auto
            _carReflection = new CarReflection(Car);
        }

        private void LoadCamara(bool teleport)
        {
            if (!Cam.Enable) 
                return;
            // Extraigo los ejes del auto de su matriz transformación
            Vector3 carPosition = Car.GetPosition();
            Vector3 z = Car.ZAxis();
            Vector3 y = Car.YAxis();
            Vector3 x = Car.XAxis();

            // Seteo la cámara en función de la posición del camara
            Vector3 camera = carPosition + Cam.ActualCamara.Y*y + Cam.ActualCamara.Z*z;
            Vector3 target = carPosition + Cam.ActualCamara.Y*y;

            /*Vector3 camera = carPosition + CAM_DELTA.Y * y + CAM_DELTA.Z * z;
                Vector3 target = carPosition + CAM_DELTA.Y * y;
                */
            Cam.SetCenterTargetUp(camera, target, new Vector3(0, 1, 0), teleport);
            GuiController.Instance.CurrentCamera = Cam;
        }

        public void DoPhysics(float elapsedTime)
        {
            TgcD3dInput input = GuiController.Instance.D3dInput;

            //Cambiar camara
            if (input.keyDown(Key.C))
            {
                Cam.ChangeCamara();
            }

            float delta_t = elapsedTime * 3;

            Car.ControlVehicle(input, delta_t);
        }

        public void RenderPlayer(float elapsedTime, List<NosBottle> checkpoints)
        {
            // Render scenario
            Piso.render();
            Car.Render();
            
            foreach (NosBottle checkpoint in checkpoints)
            {
                if (Collision.TestOBB_Vs_OBB(Car.OBB, checkpoint.Obb))
                {
                    _nosRecolectedSound.Play();
                    checkpoint.Enable = false;
                    NosCount++;
                }
            }
            
            checkpoints.ForEach(checkpoint => checkpoint.Render());
            LightAndReflection();
            LoadCamara(false);
        }

        private void LightAndReflection()
        {
            if(_playerNumber==1)
                _carReflection.Render();
        }

        private static TgcSceneLoader _loader;
        private static TgcSceneLoader Loader        
        {
            get
            {
                if(_loader==null)
                    _loader = new TgcSceneLoader();
                return _loader;
            }
        }

        public Vehicle Car { get; internal set; }
        public Camara Cam { get; internal set; }
        public TgcBox Piso { get; internal set; }
        private readonly Vector3 _camDelta = new Vector3(0, 0, 0);
        private readonly CarReflection _carReflection;
        private int _playerNumber;
        private Sound.LATSound _nosRecolectedSound;
        public int NosCount { get; set; }

    }

}