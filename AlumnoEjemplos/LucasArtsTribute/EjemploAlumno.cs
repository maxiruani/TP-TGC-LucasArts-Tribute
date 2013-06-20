using System;
using System.Collections.Generic;
using System.Drawing;
using AlumnoEjemplos.LucasArtsTribute.Circuit;
using AlumnoEjemplos.LucasArtsTribute.VehicleModel;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;
using Microsoft.DirectX.DirectInput;
using TgcViewer;
using TgcViewer.Example;
using TgcViewer.Utils.Input;
using TgcViewer.Utils.Sound;
using TgcViewer.Utils.TgcGeometry;
using TgcViewer.Utils.TgcSceneLoader;
using Device = Microsoft.DirectX.Direct3D.Device;

namespace AlumnoEjemplos.LucasArtsTribute
{
    /// <summary>
    ///     Ejemplo del alumno
    /// </summary>
    public class LucasArtsTribute : TgcExample
    {
        private CarReflection _carReflection;
        private List<Obstacle> _obstacles;
        private int _originalHeight;
        private int _originalWidth;
        public Camara cam;
        // Model car;
        private Vehicle car;
        private TgcBox lightBox;
        private List<TgcBox> obstaculos;
        private TgcBox piso;
        private List<Tgc3dSound> sonidos;

        /// <summary>
        ///     Categoría a la que pertenece el ejemplo.
        ///     Influye en donde se va a haber en el árbol de la derecha de la pantalla.
        /// </summary>
        public override string getCategory()
        {
            return "AlumnoEjemplos";
        }

        /// <summary>
        ///     Completar nombre del grupo en formato Grupo NN
        /// </summary>
        public override string getName()
        {
            return "LucasArts Tribute";
        }

        /// <summary>
        ///     Completar con la descripción del TP
        /// </summary>
        public override string getDescription()
        {
            return "Fisica del auto.";
        }

        /// <summary>
        ///     Método que se llama una sola vez,  al principio cuando se ejecuta el ejemplo.
        ///     Escribir aquí todo el código de inicialización: cargar modelos, texturas, modifiers, uservars, etc.
        ///     Borrar todo lo que no haga falta
        /// </summary>
        public override void init()
        {
            //GuiController.Instance: acceso principal a todas las herramientas del Framework
            Device d3dDevice = GuiController.Instance.D3dDevice;
            _originalWidth = d3dDevice.Viewport.Width;
            _originalHeight = d3dDevice.Viewport.Height;
            //Crear piso
            TgcTexture pisoTexture = TgcTexture.createTexture(d3dDevice,
                                                              GuiController.Instance.ExamplesMediaDir +
                                                              "Texturas\\tierra.jpg");
            piso = TgcBox.fromSize(new Vector3(0, -60, 0), new Vector3(5000, 5, 5000), pisoTexture);

            //Cargar obstaculos y posicionarlos. Los obstáculos se crean con TgcBox en lugar de cargar un modelo.
            _obstacles = new List<Obstacle>();
            sonidos = new List<Tgc3dSound>();
            Tgc3dSound sound;

            Obstacle wheelBox = new Wheel(d3dDevice, new Vector3(-50, 0, -920), new Vector3(50, 50, 50));
            //Obstaculo 1
            _obstacles.Add(wheelBox);

            _players = new List<Player>();
            
            SetCarParameters(1);

            //Ejecutar en loop los sonidos
            foreach (Tgc3dSound s in sonidos)
            {
                s.play(true);
            }
        }

        private void SetCarParameters(int playersCount)
        {
            for (int i = 0; i < playersCount; i++)
            {
                String config = GuiController.Instance.AlumnoEjemplosMediaDir + "LucasArtsTribute\\Cars\\C5\\C5.txt";
                Player player = new Player(piso, config, "Player" + i);
                _players.Add(player);
            }
            // GuiController.Instance.CurrentCamera = cam;

            /*
             * Se configura el reflejo sobre el auto. (CarReflection)
             * Se crea un Box para que simule ser el sol. Hay que mejorar esto.
            */
            //Reflejo en el auto
            /*_carReflection = new CarReflection(car);
            _carReflection.Render();
            //Crear caja para indicar ubicacion de la luz
            lightBox = TgcBox.fromSize(new Vector3(100, 100, 100), Color.Yellow);*/
        }



        /// <summary>
        ///     Método que se llama cada vez que hay que refrescar la pantalla.
        ///     Escribir aquí todo el código referido al renderizado.
        ///     Borrar todo lo que no haga falta
        /// </summary>
        /// <param name="elapsedTime">Tiempo en segundos transcurridos desde el último frame</param>
        public override void render(float elapsedTime)
        {
            //Device de DirectX para renderizar
            Device d3dDevice = GuiController.Instance.D3dDevice;
            // Vector3 lightPosition = (Vector3)GuiController.Instance.Modifiers["LightPosition"];
            if (_players.Count == 2)
            {
                LeftViewPort(elapsedTime, d3dDevice);
                RightViewPort(elapsedTime, d3dDevice);
            }
            if (_players.Count == 1)
            {
                _players[0].RenderPlayer(elapsedTime);
            }
            // car.Instrumental.GetValues().ForEach(item => item.render());
        }



        private void RightViewPort(float elapsedTime, Device d3dDevice)
        {
            d3dDevice.EndScene();
            // Set up view-port properties
            var rightViewPort = new Viewport();
            rightViewPort.X = 0;
            rightViewPort.Y = _originalHeight;
            rightViewPort.Width = _originalWidth;
            rightViewPort.Height = _originalHeight/2;
            rightViewPort.MinZ = 0.0f;
            rightViewPort.MaxZ = 1.0f;

            d3dDevice.Viewport = rightViewPort;

            // Now we can clear just view-port's portion of the buffer to green...
            d3dDevice.Clear(ClearFlags.Target | ClearFlags.ZBuffer,
                            Color.FromArgb(255, 0, 255, 0), 1.0f, 0);

            d3dDevice.BeginScene();
        }

        private void LeftViewPort(float elapsedTime, Device d3dDevice)
        {
            d3dDevice.EndScene();
            var leftViewPort = new Viewport();
            leftViewPort.X = 0;
            leftViewPort.Y = 0;
            leftViewPort.Width = _originalWidth;
            leftViewPort.Height = _originalHeight/2;
            leftViewPort.MinZ = 0.0f;
            leftViewPort.MaxZ = 1.0f;

            d3dDevice.Viewport = leftViewPort;
            // Now we can clear just view-port's portion of the buffer to green...
            d3dDevice.Clear(ClearFlags.Target | ClearFlags.ZBuffer,
                            Color.FromArgb(255, 0, 255, 0), 1.0f, 0);
            d3dDevice.BeginScene();
            TgcD3dInput input = GuiController.Instance.D3dInput;

            // ProcessInput(input);

            //Cambiar camara
            if (input.keyDown(Key.C))
            {
                cam.ChangeCamara();
            }

            float delta_t = elapsedTime*5;

            // car.DoPhysics(delta_t);

            car.ControlVehicle(input, delta_t);

            // Render piso
            piso.render();

            LightAndReflection();

            car.Render();
            //LoadCamara(false);

            // Render obstaculos
            /*
            Vector3 lastPos = car.Mesh.Position;
            
            Vector lastPos2 = car.position_wc;
            bool collitionTrue;
            
            foreach (Obstacle obstacle in _obstacles)
            {
                obstacle.Render();
                TgcCollisionUtils.BoxBoxResult result = TgcCollisionUtils.classifyBoxBox(obstacle.ObstacleBox.BoundingBox, car.Mesh.BoundingBox);
                if (result == TgcCollisionUtils.BoxBoxResult.Adentro || result == TgcCollisionUtils.BoxBoxResult.Atravesando)
                {
                    Vector3 collisionZone = TgcCollisionUtils.closestPointAABB(car.Mesh.Position, obstacle.ObstacleBox.BoundingBox);
                    car.Mesh.Position = lastPos;
                    car.Position_wc = lastPos2;
                    car.CrashWithObject();
                    break;
                }
            }
            */
        }

        private void LightAndReflection()
        {
//lightBox representa a la fuente de luz. Habría que colocarlo en algún angulo superior. Tal vez no sería necesario mostrarlo.
            var lightPosition = (Vector3) GuiController.Instance.Modifiers["LightPosition"];
            lightBox.Position = lightPosition;
            lightBox.render();
            _carReflection.Render();
        }

        private void CheckCollisionPos()
        {
        }

        /// <summary>
        ///     Método que se llama cuando termina la ejecución del ejemplo.
        ///     Hacer dispose() de todos los objetos creados.
        /// </summary>
        public override void close()
        {
            piso.dispose();
            foreach (Obstacle obstacle in _obstacles)
            {
                obstacle.Dispose();
            }

            car.body.dispose();

            foreach (Tgc3dSound sound in sonidos)
            {
                sound.dispose();
            }
        }

        private List<Player> _players;
    }
}