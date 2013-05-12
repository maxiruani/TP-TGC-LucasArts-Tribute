using System;
using System.Collections.Generic;
using System.Text;
using System.Drawing;
using AlumnoEjemplos.LucasArtsTribute.Circuit;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;
using Microsoft.DirectX.DirectInput;
using TgcViewer;
using TgcViewer.Example;
using TgcViewer.Utils.Modifiers;
using TgcViewer.Utils.Terrain;
using TgcViewer.Utils.TgcSceneLoader;
using TgcViewer.Utils.TgcGeometry;
using TgcViewer.Utils.Input;
using TgcViewer.Utils.TgcSkeletalAnimation;
using TgcViewer.Utils.Sound;

using AlumnoEjemplos.LucasArtsTribute;
using AlumnoEjemplos.LucasArtsTribute.Models;

namespace AlumnoEjemplos.LucasArtsTribute
{
    /// <summary>
    /// Ejemplo del alumno
    /// </summary>
    public class LucasArtsTribute : TgcExample
    {

        TgcBox piso;
        List<TgcBox> obstaculos;
        private List<Obstacle> _obstacles;
        List<Tgc3dSound> sonidos;
        CorvetteCar car;

        /// <summary>
        /// Categoría a la que pertenece el ejemplo.
        /// Influye en donde se va a haber en el árbol de la derecha de la pantalla.
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
        /// Completar con la descripción del TP
        /// </summary>
        public override string getDescription()
        {
            return "Fisica del auto.";
        }

        /// <summary>
        /// Método que se llama una sola vez,  al principio cuando se ejecuta el ejemplo.
        /// Escribir aquí todo el código de inicialización: cargar modelos, texturas, modifiers, uservars, etc.
        /// Borrar todo lo que no haga falta
        /// </summary>
        public override void init()
        {
            //GuiController.Instance: acceso principal a todas las herramientas del Framework
            Microsoft.DirectX.Direct3D.Device d3dDevice = GuiController.Instance.D3dDevice;
            //Crear piso
            TgcTexture pisoTexture = TgcTexture.createTexture(d3dDevice, GuiController.Instance.ExamplesMediaDir + "Texturas\\tierra.jpg");
            piso = TgcBox.fromSize(new Vector3(0, -60, 0), new Vector3(5000, 5, 5000), pisoTexture);

            //Cargar obstaculos y posicionarlos. Los obstáculos se crean con TgcBox en lugar de cargar un modelo.
            _obstacles = new List<Obstacle>();
            sonidos = new List<Tgc3dSound>();
            Tgc3dSound sound;

            Obstacle wheelBox = new Wheel(d3dDevice, new Vector3(-50, 0, -920), new Vector3(50, 50, 50));
            //Obstaculo 1
            _obstacles.Add(wheelBox);

            //Cargar personaje principal
            TgcSceneLoader loader = new TgcSceneLoader();
            TgcScene scene = loader.loadSceneFromFile(GuiController.Instance.ExamplesMediaDir + "MeshCreator\\Meshes\\Vehiculos\\Hummer\\Hummer-TgcScene.xml");

            car = new CorvetteCar();
            car.Mesh = scene.Meshes[0];
            car.Mesh.Position = new Vector3(0, -50, 0);
            new CarReflection().EffectEnable(car);
            
            
            //Crear caja para indicar ubicacion de la luz
            lightBox = TgcBox.fromSize(new Vector3(5, 5, 5), Color.Yellow);
            

            //Hacer que el Listener del sonido 3D siga al personaje
            GuiController.Instance.DirectSound.ListenerTracking = car.Mesh;

            //Configurar camara en Tercer Persona
            /*
            GuiController.Instance.ThirdPersonCamera.Enable = true;
            GuiController.Instance.ThirdPersonCamera.setCamera(car.Position, 200, 300);
            GuiController.Instance.ThirdPersonCamera.TargetDisplacement = new Vector3(0, 100, 0);
            */
            // Crear la cámara
            cam = new Camara();
            cam.SetCenterTargetUp(CAM_DELTA, new Vector3(0, 0, 0), new Vector3(0, 1, 0), true);
            cam.Enable = true;
            GuiController.Instance.CurrentCamera = cam;
            LoadCamara(true);

            

            //Ejecutar en loop los sonidos
            foreach (Tgc3dSound s in sonidos)
            {
                s.play(true);
            }

        }

        private void LoadCamara(bool teleport)
        {
            // Extraigo los ejes del avion de su matriz transformación
            Vector3 carPosition = car.GetPosition();
            Vector3 z = car.ZAxis();
            Vector3 y = car.YAxis();
            Vector3 x = car.XAxis();

            // Seteo la cámara en función de la posición del avion
            Vector3 camera = carPosition + cam.ActualCamara.Y * y + cam.ActualCamara.Z * z;
            Vector3 target = carPosition + cam.ActualCamara.Y * y;
            
            /*Vector3 camera = carPosition + CAM_DELTA.Y * y + CAM_DELTA.Z * z;
            Vector3 target = carPosition + CAM_DELTA.Y * y;
            */cam.SetCenterTargetUp(camera, target, new Vector3(0, 1, 0), teleport);
        }

        public void ProcessInput(TgcD3dInput input)
        {
            // Acelerador
            if (input.keyDown(Key.W))
            {
                this.car.throttle = 1;
                this.car.brake = 0;
            }
            else if (input.keyDown(Key.S))
            {
                this.car.brake = 0;
                this.car.throttle = 0;
            }
            else
            {
                this.car.throttle = 0;
                this.car.brake = 0;
            }
        }

        /// <summary>
        /// Método que se llama cada vez que hay que refrescar la pantalla.
        /// Escribir aquí todo el código referido al renderizado.
        /// Borrar todo lo que no haga falta
        /// </summary>
        /// <param name="elapsedTime">Tiempo en segundos transcurridos desde el último frame</param>
        public override void render(float elapsedTime)
        {
            //Device de DirectX para renderizar
            Microsoft.DirectX.Direct3D.Device d3dDevice = GuiController.Instance.D3dDevice;
            Vector3 lightPosition = (Vector3)GuiController.Instance.Modifiers["LightPosition"];

            TgcD3dInput input = GuiController.Instance.D3dInput;

            ProcessInput(input);

            //car.Move(d3dInput, elapsedTime);

            //Cambiar camara
            if (input.keyDown(Key.C))
            {
                cam.ChangeCamara();
            }

            float delta_t = elapsedTime * 10;

            car.DoPhysics(delta_t);

            // Render piso
            piso.render();
            
            // Render obstaculos
            Vector3 lastPos = car.Mesh.Position;
            Vector lastPos2 = car.Position_wc;
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

            //Mover mesh que representa la luz
            lightBox.Position = lightPosition;
            // Render del Auto
            car.Render();
            LoadCamara(false);
            car.Instrumental.GetValues().ForEach(item => item.render());
        }

        private TgcBox lightBox;

        private void CheckCollisionPos()
        {
            
        }

        /// <summary>
        /// Método que se llama cuando termina la ejecución del ejemplo.
        /// Hacer dispose() de todos los objetos creados.
        /// </summary>
        public override void close()
        {
            piso.dispose();
            foreach (Obstacle obstacle in _obstacles)
            {
                obstacle.Dispose();
            }
            car.Mesh.dispose();

            foreach (Tgc3dSound sound in sonidos)
            {
                sound.dispose();
            }
        }


        public Camara cam;
        Vector3 CAM_DELTA = new Vector3(0, 50, 250);


    }
}
