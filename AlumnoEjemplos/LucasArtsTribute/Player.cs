using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using AlumnoEjemplos.LucasArtsTribute.VehicleModel;
using Microsoft.DirectX;
using Microsoft.DirectX.DirectInput;
using TgcViewer;
using TgcViewer.Utils.Input;
using TgcViewer.Utils.TgcGeometry;

namespace AlumnoEjemplos.LucasArtsTribute
{
    class Player
    {
        public Player(TgcBox piso, String carConfiguration, String playerName)
        {
            this._playerName = playerName;
            Car = new Vehicle(carConfiguration);
            Piso = piso;
            Cam = new Camara();
            Cam.SetCenterTargetUp(_camDelta, new Vector3(0, 0, 0), new Vector3(0, 1, 0), true);
            Cam.Enable = true;
            LoadCamara(true);
        }

        private void LoadCamara(bool teleport)
        {
            // Extraigo los ejes del auto de su matriz transformación
            Vector3 carPosition = Car.GetPosition();
            Vector3 z = Car.ZAxis();
            Vector3 y = Car.YAxis();
            Vector3 x = Car.XAxis();

            // Seteo la cámara en función de la posición del avion
            Vector3 camera = carPosition + Cam.ActualCamara.Y * y + Cam.ActualCamara.Z * z;
            Vector3 target = carPosition + Cam.ActualCamara.Y * y;

            /*Vector3 camera = carPosition + CAM_DELTA.Y * y + CAM_DELTA.Z * z;
            Vector3 target = carPosition + CAM_DELTA.Y * y;
            */
            Cam.SetCenterTargetUp(camera, target, new Vector3(0, 1, 0), teleport);
            GuiController.Instance.CurrentCamera = Cam;
        }

        public void RenderPlayer(float elapsedTime)
        {
            TgcD3dInput input = GuiController.Instance.D3dInput;
            //GuiController.Instance.CurrentCamera = Cam;
            //Cambiar camara
            if (input.keyDown(Key.C))
            {
                Cam.ChangeCamara();
            }

            float delta_t = elapsedTime * 5;

            Car.ControlVehicle(input, delta_t);

            // Render piso
            Piso.render();

            //LightAndReflection();

            Car.Render();
            LoadCamara(false);
        }

        private String _playerName ;
        public Vehicle Car { get; internal set; }
        public Camara Cam { get; internal set; }
        public TgcBox Piso { get; internal set; }
        private readonly Vector3 _camDelta = new Vector3(0, 50, 250);

    }
}
