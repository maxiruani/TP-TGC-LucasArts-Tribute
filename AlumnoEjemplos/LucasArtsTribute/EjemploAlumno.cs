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
            
            SetCarParameters(2);

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
                Player player = new Player(piso, config, new Vector3(5,0,5), 1); //Envío la superficie, la configuración y la posicion inicial
                _players.Add(player);
            }
            else if (playersCount == 2)
            {
                String config1 = GuiController.Instance.AlumnoEjemplosMediaDir + "LucasArtsTribute\\Cars\\C5\\C5.txt";
                String config2 = GuiController.Instance.AlumnoEjemplosMediaDir + "LucasArtsTribute\\Cars\\TT\\TT.txt";
                _players = Multiplayer.CreateMultiplayer(piso, config1, config2);
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
                //_players[0].Cam.Enable = true;
                _players[0].RenderPlayer(elapsedTime);
            }
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