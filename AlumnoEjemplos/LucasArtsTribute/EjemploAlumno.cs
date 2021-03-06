using System;
using System.Collections.Generic;
using System.Drawing;
using System.Threading;
using AlumnoEjemplos.LucasArtsTribute.Circuit;
using AlumnoEjemplos.LucasArtsTribute.Sound;
using AlumnoEjemplos.LucasArtsTribute.VehicleModel;
using Microsoft.DirectX;
using Microsoft.DirectX.DirectInput;
using TgcViewer;
using TgcViewer.Example;
using TgcViewer.Utils.Sound;
using TgcViewer.Utils.Terrain;
using TgcViewer.Utils.TgcGeometry;
using TgcViewer.Utils.TgcSceneLoader;
using TgcViewer.Utils._2D;
using Device = Microsoft.DirectX.Direct3D.Device;

namespace AlumnoEjemplos.LucasArtsTribute
{
    /// <summary>
    ///     Ejemplo del alumno
    /// </summary>
    public class LucasArtsTribute : TgcExample
    {
        private static int NumberOfPlayers = 1;
        private readonly String _alumnoMediaFolder = GuiController.Instance.AlumnoEjemplosMediaDir;
        private bool _isGameStarted;
        private List<NosBottle> _nosBottles;

        private List<Player> _players;
        private TgcSkyBox _skyBox;        public Camara cam;

        private Vehicle car;
        private List<TgcBox> obstaculos;        private DateTime presentationDateTime = DateTime.Now;
        private TgcBox scenario;
        private List<Tgc3dSound> sonidos;

        #region Presentation Fields

        private bool _presentationLoad;
        private bool _showingPresentation = true;
        private TgcSprite presentationImage;
        private TgcText2d presentationText;
        private Mp3 sound;

        #endregion


        /// <summary>
        ///     Categor�a a la que pertenece el ejemplo.
        ///     Influye en donde se va a haber en el �rbol de la derecha de la pantalla.
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
        ///     Completar con la descripci�n del TP
        /// </summary>
        public override string getDescription()
        {
            return "Fisica del auto.";
        }

        /// <summary>
        ///     M�todo que se llama una sola vez,  al principio cuando se ejecuta el ejemplo.
        ///     Escribir aqu� todo el c�digo de inicializaci�n: cargar modelos, texturas, modifiers, uservars, etc.
        ///     Borrar todo lo que no haga falta
        /// </summary>
        public override void init()
        {
            //GuiController.Instance: acceso principal a todas las herramientas del Framework
            Device d3dDevice = GuiController.Instance.D3dDevice;
            //Cargar scenario
            LoadScenario();
            //Cargo SkyBox
            LoadSkyBox();

            //Cargar obstaculos y posicionarlos. Los obst�culos se crean con TgcBox en lugar de cargar un modelo.
            sonidos = new List<Tgc3dSound>();
            Tgc3dSound sound;

            //Obstaculo 1

            _players = new List<Player>();

            SetCarParameters(NumberOfPlayers);

            //Ejecutar en loop los sonidos
            foreach (Tgc3dSound s in sonidos)
            {
                s.play(true);
            }
        }

        private void SetCarParameters(int playersCount)
        {
            if (playersCount == 1)
            {
                String config = GuiController.Instance.AlumnoEjemplosMediaDir + "LucasArtsTribute\\Cars\\C5\\C5.txt";
                var player = new Player(scenario, config, new Vector3(5, -50, 5), 1, false);
                    //Env�o la superficie, la configuraci�n y la posicion inicial
                _players.Add(player);
            }
            else if (playersCount == 2)
            {
                String config1 = GuiController.Instance.AlumnoEjemplosMediaDir + "LucasArtsTribute\\Cars\\TT\\TT.txt";
                String config2 = GuiController.Instance.AlumnoEjemplosMediaDir + "LucasArtsTribute\\Cars\\MC\\MC.txt";
                _players = Multiplayer.CreateMultiplayer(scenario, config1, config2, _skyBox, _nosBottles);
            }
            // GuiController.Instance.CurrentCamera = cam;
        }


        /// <summary>
        ///     M�todo que se llama cada vez que hay que refrescar la pantalla.
        ///     Escribir aqu� todo el c�digo referido al renderizado.
        ///     Borrar todo lo que no haga falta
        /// </summary>
        /// <param name="elapsedTime">Tiempo en segundos transcurridos desde el �ltimo frame</param>
        public override void render(float elapsedTime)
        {
            if (!_isGameStarted)
            {
                while (_showingPresentation &&
                       !(GuiController.Instance.D3dInput.keyDown(Key.D1) ||
                         GuiController.Instance.D3dInput.keyDown(Key.D2)))
                {
                    Presentation();
                    return;
                }

                sound.Stop();
                if (_showingPresentation)
                {
                    if (GuiController.Instance.D3dInput.keyDown(Key.D1))
                        NumberOfPlayers = 1;
                    else if (GuiController.Instance.D3dInput.keyDown(Key.D2))
                        NumberOfPlayers = 2;
                    else
                        return;
                    init();
                    _showingPresentation = false;
                    _isGameStarted = true;
                    return;
                }
            }



            //Device de DirectX para renderizar
            Device d3dDevice = GuiController.Instance.D3dDevice;
            // Vector3 lightPosition = (Vector3)GuiController.Instance.Modifiers["LightPosition"];
            if (_players.Count == 2)
            {
                Multiplayer.RenderAll(_players);
            }
            if (_players.Count == 1)
            {
                _players[0].DoPhysics(elapsedTime);
                _players[0].RenderPlayer(elapsedTime, _nosBottles);
                _skyBox.render();
                _nosBottles.ForEach(item => item.Render());
            }
        }


        private void Presentation()
        {
            var textPosition = new Point(200, 100);
            if (_presentationLoad == false)
            {
                presentationImage = new TgcSprite();
                presentationImage.Texture =
                    TgcTexture.createTexture(GuiController.Instance.AlumnoEjemplosMediaDir +
                                             "LucasArtsTribute\\Presentation\\AudiTT.jpg");
                presentationImage.Position = new Vector2(0, 0);
                presentationImage.Scaling = new Vector2(0.6f, 0.6f);

                presentationText = new TgcText2d();
                presentationText.Color = Color.OrangeRed;
                presentationText.Text = "Presione 1 para un player o 2 para dos players.";
                presentationText.Size = new Size(500, 400);
                presentationText.changeFont(new Font("Times New Roman", 25.0f));
                presentationText.Position = textPosition;
                sound = new Mp3("LucasArtsTribute\\Presentation\\Daytona.mp3");
                _presentationLoad = true;
            }
            if ((DateTime.Now - presentationDateTime).Seconds > 2)
            {
                presentationText.Position = NewPosition();
                presentationDateTime = DateTime.Now;
            }

            GuiController.Instance.Drawer2D.beginDrawSprite();
            presentationImage.render();
            GuiController.Instance.Drawer2D.endDrawSprite();
            presentationText.render();

            if (!sound.IsPlaying())
                sound.Play();
        }

        private Point NewPosition()
        {
            var rand = new Random(DateTime.Now.Millisecond);
            int xPoint = rand.Next(0, 400);
            int yPoint = rand.Next(0, 400);
            return new Point(xPoint, yPoint);
        }


        public void LoadScenario()
        {
            Device d3DDevice = GuiController.Instance.D3dDevice;
            TgcTexture pisoTexture = TgcTexture.createTexture(d3DDevice,
                                                              _alumnoMediaFolder + "LucasArtsTribute\\circuito.jpg");
            scenario = TgcBox.fromSize(new Vector3(0, -60, 0), new Vector3(5000, 5, 5000), pisoTexture);
            _nosBottles = Checkpoint.CreateAllCheckPoints();
        }


        public void LoadSkyBox()
        {
            string texturesPath = _alumnoMediaFolder + "LucasArtsTribute\\Texturas\\SkyBox1\\";

            //Crear SkyBox 
            _skyBox = new TgcSkyBox();
            _skyBox.Center = new Vector3(0, -60, 0);
            _skyBox.Size = new Vector3(10000, 5000, 10000);

            //Configurar las texturas para cada una de las 6 caras
            _skyBox.setFaceTexture(TgcSkyBox.SkyFaces.Up, texturesPath + "phobos_up.jpg");
            _skyBox.setFaceTexture(TgcSkyBox.SkyFaces.Down, texturesPath + "phobos_dn.jpg");
            _skyBox.setFaceTexture(TgcSkyBox.SkyFaces.Left, texturesPath + "phobos_lf.jpg");
            _skyBox.setFaceTexture(TgcSkyBox.SkyFaces.Right, texturesPath + "phobos_rt.jpg");

            //Hay veces es necesario invertir las texturas Front y Back si se pasa de un sistema RightHanded a uno LeftHanded
            _skyBox.setFaceTexture(TgcSkyBox.SkyFaces.Front, texturesPath + "phobos_bk.jpg");
            _skyBox.setFaceTexture(TgcSkyBox.SkyFaces.Back, texturesPath + "phobos_ft.jpg");

            //Actualizar todos los valores para crear el SkyBox
            _skyBox.updateValues();
        }


        private void CheckCollisionPos()
        {
        }

        /// <summary>
        ///     M�todo que se llama cuando termina la ejecuci�n del ejemplo.
        ///     Hacer dispose() de todos los objetos creados.
        /// </summary>
        public override void close()
        {
            scenario.dispose();
            foreach (Checkpoint obstacle in _nosBottles)
            {
                obstacle.Dispose();
            }

            //car.body.dispose();

            foreach (Tgc3dSound sound in sonidos)
            {
                sound.dispose();
            }
        }
    }
}