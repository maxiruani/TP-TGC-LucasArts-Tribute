using System;
using AlumnoEjemplos.LucasArtsTribute.Players;
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
        public Player(TgcBox piso, String carConfiguration, Vector3 initialPosition, int numberOfPlayer)
        {
            Car = new Vehicle(carConfiguration, initialPosition, Loader, UserControlsFactory.Create(numberOfPlayer));
            Piso = piso;
            Cam = new Camara(initialPosition);
            Cam.SetCenterTargetUp(_camDelta, initialPosition, new Vector3(0, 100, 0), true); 
            Cam.Enable = true;
            LoadCamara(true);
            /*
             * Se configura el reflejo sobre el auto. (CarReflection)
             * Se crea un Box para que simule ser el sol. Hay que mejorar esto.
             */
            //Reflejo en el auto
            /*_carReflection = new CarReflection(Car);
            _carReflection.Render();*/
            /*//Crear caja para indicar ubicacion de la luz
            lightBox = TgcBox.fromSize(new Vector3(100, 100, 100), Color.Yellow);*/
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

            float delta_t = elapsedTime * 5;

            Car.ControlVehicle(input, delta_t);
        }

        public void RenderPlayer(float elapsedTime)
        {
            // Render piso
            Piso.render();
            Car.Render();
            LightAndReflection();
            LoadCamara(false);
        }

        private void LightAndReflection()
        {
            //lightBox representa a la fuente de luz. Habría que colocarlo en algún angulo superior. Tal vez no sería necesario mostrarlo.
            //     var lightPosition = (Vector3)GuiController.Instance.Modifiers["LightPosition"];
            //     lightBox.Position = lightPosition;
            //     lightBox.render();
           // _carReflection.Render();
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
    }

}