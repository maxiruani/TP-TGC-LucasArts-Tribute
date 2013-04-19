using System;
using System.Collections.Generic;
using System.Text;
using System.Drawing;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;
using Microsoft.DirectX.DirectInput;
using TgcViewer;
using TgcViewer.Example;
using TgcViewer.Utils.Modifiers;
using TgcViewer.Utils.TgcSceneLoader;
using TgcViewer.Utils.TgcGeometry;
using TgcViewer.Utils.Input;
using TgcViewer.Utils.TgcSkeletalAnimation;
using TgcViewer.Utils.Sound;

using AlumnoEjemplos.LucasArtsTribute;


namespace AlumnoEjemplos.LucasArtsTribute
{
    /// <summary>
    /// Ejemplo del alumno
    /// </summary>
    public class LucasArtsTribute : TgcExample
    {

        TgcBox piso;
        List<TgcBox> obstaculos;
        List<Tgc3dSound> sonidos;
        Car car;

        /// <summary>
        /// Categor�a a la que pertenece el ejemplo.
        /// Influye en donde se va a haber en el �rbol de la derecha de la pantalla.
        /// </summary>
        public override string getCategory()
        {
            return "AlumnoEjemplos";
        }

        /// <summary>
        /// Completar nombre del grupo en formato Grupo NN
        /// </summary>
        public override string getName()
        {
            return "LucasArts Tribute";
        }

        /// <summary>
        /// Completar con la descripci�n del TP
        /// </summary>
        public override string getDescription()
        {
            return "Fisica del auto.";
        }

        /// <summary>
        /// M�todo que se llama una sola vez,  al principio cuando se ejecuta el ejemplo.
        /// Escribir aqu� todo el c�digo de inicializaci�n: cargar modelos, texturas, modifiers, uservars, etc.
        /// Borrar todo lo que no haga falta
        /// </summary>
        public override void init()
        {
            //GuiController.Instance: acceso principal a todas las herramientas del Framework
            Microsoft.DirectX.Direct3D.Device d3dDevice = GuiController.Instance.D3dDevice;

            //Crear piso
            TgcTexture pisoTexture = TgcTexture.createTexture(d3dDevice, GuiController.Instance.ExamplesMediaDir + "Texturas\\tierra.jpg");
            piso = TgcBox.fromSize(new Vector3(0, -60, 0), new Vector3(5000, 5, 5000), pisoTexture);

            //Cargar obstaculos y posicionarlos. Los obst�culos se crean con TgcBox en lugar de cargar un modelo.
            obstaculos = new List<TgcBox>();
            sonidos = new List<Tgc3dSound>();
            TgcBox obstaculo;
            Tgc3dSound sound;

            //Obstaculo 1
            obstaculo = TgcBox.fromSize(
                new Vector3(-200, 0, 0),
                new Vector3(80, 150, 80),
                TgcTexture.createTexture(d3dDevice, GuiController.Instance.ExamplesMediaDir + "Texturas\\Quake\\TexturePack3\\goo2.jpg"));
            obstaculos.Add(obstaculo);

            //Sondio obstaculo 1
            //OJO, solo funcionan sonidos WAV Mono (No stereo). Hacer boton der => Propiedades sobre el archivo
            //y tiene que decir "1 Channel".
            sound = new Tgc3dSound(GuiController.Instance.ExamplesMediaDir + "Sound\\armon�a, continuo.wav", obstaculo.Position);
            //Hay que configurar la m�nima distancia a partir de la cual se empieza a atenuar el sonido 3D
            sound.MinDistance = 50f;
            sonidos.Add(sound);

            //Obstaculo 2
            obstaculo = TgcBox.fromSize(
                new Vector3(200, 0, 800),
                new Vector3(80, 300, 80),
                TgcTexture.createTexture(d3dDevice, GuiController.Instance.ExamplesMediaDir + "Texturas\\Quake\\TexturePack3\\lun_dirt.jpg"));
            obstaculos.Add(obstaculo);

            //Sondio obstaculo 2
            sound = new Tgc3dSound(GuiController.Instance.ExamplesMediaDir + "Sound\\viento helado.wav", obstaculo.Position);
            sound.MinDistance = 50f;
            sonidos.Add(sound);

            //Obstaculo 3
            obstaculo = TgcBox.fromSize(
                new Vector3(600, 0, 400),
                new Vector3(80, 100, 150),
                TgcTexture.createTexture(d3dDevice, GuiController.Instance.ExamplesMediaDir + "Texturas\\Quake\\TexturePack3\\Metal2_1.jpg"));
            obstaculos.Add(obstaculo);

            //Sondio obstaculo 3
            sound = new Tgc3dSound(GuiController.Instance.ExamplesMediaDir + "Sound\\risa de man�aco.wav", obstaculo.Position);
            sound.MinDistance = 50f;
            sonidos.Add(sound);


            //Cargar personaje principal
            TgcSceneLoader loader = new TgcSceneLoader();
            TgcScene scene = loader.loadSceneFromFile(GuiController.Instance.ExamplesMediaDir + "MeshCreator\\Meshes\\Vehiculos\\Hummer\\Hummer-TgcScene.xml");
            
            car = new Car(scene.Meshes[0]);
            car.Position = new Vector3(0, -50, 0);
            car.Obstacles = obstaculos;
            

            //Hacer que el Listener del sonido 3D siga al personaje
            GuiController.Instance.DirectSound.ListenerTracking = car.Mesh;

            //Configurar camara en Tercer Persona
            GuiController.Instance.ThirdPersonCamera.Enable = true;

            GuiController.Instance.ThirdPersonCamera.setCamera(car.Position, 200, 300);

            GuiController.Instance.ThirdPersonCamera.TargetDisplacement = new Vector3(0, 100, 0);

            //Ejecutar en loop los sonidos
            foreach (Tgc3dSound s in sonidos)
            {
                s.play(true);
            }

        }

        /// <summary>
        /// M�todo que se llama cada vez que hay que refrescar la pantalla.
        /// Escribir aqu� todo el c�digo referido al renderizado.
        /// Borrar todo lo que no haga falta
        /// </summary>
        /// <param name="elapsedTime">Tiempo en segundos transcurridos desde el �ltimo frame</param>
        public override void render(float elapsedTime)
        {
            //Device de DirectX para renderizar
            Microsoft.DirectX.Direct3D.Device d3dDevice = GuiController.Instance.D3dDevice;

            TgcD3dInput d3dInput = GuiController.Instance.D3dInput;

            car.Move(d3dInput, elapsedTime);

            // Hacer que la camara siga al personaje en su nueva posicion
            GuiController.Instance.ThirdPersonCamera.Target = car.Mesh.Position;

            // Render piso
            piso.render();

            // Render obstaculos
            foreach (TgcBox obstaculo in obstaculos)
            {
                obstaculo.render();
            }
            
            // Render del Auto
            car.Mesh.render();
        }

        /// <summary>
        /// M�todo que se llama cuando termina la ejecuci�n del ejemplo.
        /// Hacer dispose() de todos los objetos creados.
        /// </summary>
        public override void close()
        {
            piso.dispose();
            foreach (TgcBox obstaculo in obstaculos)
            {
                obstaculo.dispose();
            }
            car.Mesh.dispose();

            foreach (Tgc3dSound sound in sonidos)
            {
                sound.dispose();
            }
        }

    }
}
