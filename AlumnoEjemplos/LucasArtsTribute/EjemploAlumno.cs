using System;
using System.Collections.Generic;
using System.Drawing;
using AlumnoEjemplos.LucasArtsTribute.Circuit;
using AlumnoEjemplos.LucasArtsTribute.Sound;
using AlumnoEjemplos.LucasArtsTribute.VehicleModel;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;
using Microsoft.DirectX.DirectInput;
using TgcViewer;
using TgcViewer.Example;
using TgcViewer.Utils.Input;
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
        private List<Obstacle> _obstacles;
        private int _originalHeight;
        private int _originalWidth;
        public Camara cam;
        // Model car;
        private Vehicle car;
        private TgcBox lightBox;
        private List<TgcBox> obstaculos;
        private TgcBox scenario;
        private List<Tgc3dSound> sonidos;
        private List<Player> _players;
        private const int NumberOfPlayers = 2;
        private TgcSkyBox _skyBox;
        readonly String _alumnoMediaFolder = GuiController.Instance.AlumnoEjemplosMediaDir;
        
#region Presentation Fields
        private bool _showingPresentation = true;
        private bool _presentationLoad = false;
        private TgcText2d presentationText;
        private TgcSprite presentationImage;
        private Mp3 sound;
#endregion
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
            //Cargar scenario
            LoadScenario();
            //Cargo SkyBox
            LoadSkyBox();

            //Cargar obstaculos y posicionarlos. Los obstáculos se crean con TgcBox en lugar de cargar un modelo.
            _obstacles = new List<Obstacle>();
            sonidos = new List<Tgc3dSound>();
            Tgc3dSound sound;

            Obstacle wheelBox = new Wheel(d3dDevice, new Vector3(-50, 0, -920), new Vector3(50, 50, 50));
            //Obstaculo 1
            _obstacles.Add(wheelBox);

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
            if(playersCount==1)
            {
                String config = GuiController.Instance.AlumnoEjemplosMediaDir + "LucasArtsTribute\\Cars\\C5\\C5.txt";
                Player player = new Player(scenario, config, new Vector3(5,-20,5), 1); //Envío la superficie, la configuración y la posicion inicial
                _players.Add(player);
            }
            else if (playersCount == 2)
            {
                String config1 = GuiController.Instance.AlumnoEjemplosMediaDir + "LucasArtsTribute\\Cars\\C5\\C5.txt";
                String config2 = GuiController.Instance.AlumnoEjemplosMediaDir + "LucasArtsTribute\\Cars\\TT\\TT.txt";
                _players = Multiplayer.CreateMultiplayer(scenario, config1, config2, _skyBox);
            }
            // GuiController.Instance.CurrentCamera = cam;

        }


        /// <summary>
        ///     Método que se llama cada vez que hay que refrescar la pantalla.
        ///     Escribir aquí todo el código referido al renderizado.
        ///     Borrar todo lo que no haga falta
        /// </summary>
        /// <param name="elapsedTime">Tiempo en segundos transcurridos desde el último frame</param>
        public override void render(float elapsedTime)
        {
            while (_showingPresentation && !GuiController.Instance.D3dInput.keyDown(Key.Space))
            {
                Presentation();
                return;
            }
            sound.Stop();
            _showingPresentation = false;
            
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
                _players[0].RenderPlayer(elapsedTime);
                _skyBox.render();
            }
        }

        

        private void Presentation()
        {
            if (_presentationLoad==false)
            {
                presentationImage = new TgcSprite();
                presentationImage.Texture =
                    TgcTexture.createTexture(GuiController.Instance.AlumnoEjemplosMediaDir +
                                             "LucasArtsTribute\\Presentation\\AudiTT.jpg");
                presentationImage.Position = new Vector2(0, 0);
                presentationImage.Scaling = new Vector2(0.6f, 0.6f);
                presentationText = new TgcText2d();
                presentationText.Color = Color.OrangeRed;
                presentationText.Text = "Presione espacio para comenzar.";
                presentationText.Size = new Size(100, 20);
                presentationText.Position = new Point(0, 0);
                sound = new Mp3("LucasArtsTribute\\Presentation\\Daytona.mp3");
                _presentationLoad = true;
            }
            GuiController.Instance.Drawer2D.beginDrawSprite();
            presentationImage.render();
            GuiController.Instance.Drawer2D.endDrawSprite();
            presentationText.render();

            if(!sound.IsPlaying())
                sound.Play();
            
        }


        public void LoadScenario()
        {
            Device d3DDevice = GuiController.Instance.D3dDevice;
            TgcTexture pisoTexture = TgcTexture.createTexture(d3DDevice, _alumnoMediaFolder + "LucasArtsTribute\\circuito.jpg");
            scenario = TgcBox.fromSize(new Vector3(0, -60, 0), new Vector3(5000, 5, 5000), pisoTexture);

            obstaculos = new List<TgcBox>();
            //Pared1
            TgcBox obstaculo = TgcBox.fromSize(
                new Vector3(0, 0, 2500),
                new Vector3(5000, 150, 80),
                TgcTexture.createTexture(d3DDevice, _alumnoMediaFolder + "LucasArtsTribute\\Texturas\\TexturePack2\\rock_wall.jpg"));
            obstaculos.Add(obstaculo);

            //Pared2
            obstaculo = TgcBox.fromSize(
               new Vector3(0, 0, -2500),
               new Vector3(5000, 150, 80),
               TgcTexture.createTexture(d3DDevice, _alumnoMediaFolder + "LucasArtsTribute\\Texturas\\TexturePack2\\rock_wall.jpg"));
            obstaculos.Add(obstaculo);

            //Pared3
            obstaculo = TgcBox.fromSize(
               new Vector3(2500, 0, 0),
               new Vector3(80, 150, 5000),
               TgcTexture.createTexture(d3DDevice, _alumnoMediaFolder + "LucasArtsTribute\\Texturas\\TexturePack2\\rock_wall.jpg"));
            obstaculos.Add(obstaculo);

            //Pared4
            obstaculo = TgcBox.fromSize(
               new Vector3(-2500, 0, 0),
               new Vector3(80, 150, 5000),
               TgcTexture.createTexture(d3DDevice, _alumnoMediaFolder + "LucasArtsTribute\\Texturas\\TexturePack2\\rock_wall.jpg"));
            obstaculos.Add(obstaculo);
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
        ///     Método que se llama cuando termina la ejecución del ejemplo.
        ///     Hacer dispose() de todos los objetos creados.
        /// </summary>
        public override void close()
        {
            scenario.dispose();
            foreach (Obstacle obstacle in _obstacles)
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