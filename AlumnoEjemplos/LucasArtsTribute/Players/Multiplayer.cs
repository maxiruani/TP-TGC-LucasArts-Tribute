using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;
using TgcViewer;
using TgcViewer.Utils.TgcGeometry;

namespace AlumnoEjemplos.LucasArtsTribute
{
    class Multiplayer
    {
        public static List<Player> CreateMultiplayer(TgcBox piso, String configurationCar1, String configurationCar2)
        {
            _originalWidth = GuiController.Instance.D3dDevice.Viewport.Width;
            _originalHeight = GuiController.Instance.D3dDevice.Viewport.Height;
            List<Player> players = new List<Player>();

            Vector3 initialPosition = new Vector3(100, 0, 0);
            Player player = new Player(piso, configurationCar1, initialPosition);
            players.Add(player);
            _rightViewPort = RightViewPortCreate();

            initialPosition = new Vector3(10, 0, 0);
            player = new Player(piso, configurationCar2, initialPosition);
            players.Add(player);
            _leftViewPort = LeftViewPortCreate();

            GuiController.Instance.CustomRenderEnabled = true;

            return players;
        }
        static Stopwatch  sw1 = new Stopwatch();
        static Stopwatch sw2 = new Stopwatch();

        public static void RenderAll(List<Player> players)
        {
            float elapsedTime = GuiController.Instance.ElapsedTime;
            players[0].DoPhysics(elapsedTime);
            players[1].DoPhysics(elapsedTime);

            RightViewPort(GuiController.Instance.D3dDevice, players);

            GuiController.Instance.CurrentCamera.updateCamera();
            GuiController.Instance.CurrentCamera.updateViewMatrix(GuiController.Instance.D3dDevice);

            LeftViewPort(GuiController.Instance.D3dDevice, players);
            GuiController.Instance.D3dDevice.Present();

        }


        private static void RightViewPort(Device d3dDevice, List<Player> players)
        {
            // Set up view-port properties
            float elapsedTime = GuiController.Instance.ElapsedTime;

            d3dDevice.Viewport = _rightViewPort;

            // Now we can clear just view-port's portion of the buffer to green...
            d3dDevice.Clear(ClearFlags.Target | ClearFlags.ZBuffer,
                            Color.FromArgb(255, 0, 255, 0), 1.0f, 0);

            d3dDevice.BeginScene();
            players[0].Cam.Enable = false;
            players[1].Cam.Enable = true;
            players[0].RenderPlayer(elapsedTime);
            players[1].RenderPlayer(elapsedTime);
            d3dDevice.EndScene();

        }

        private static Object thisLock = new Object();

        private static void LeftViewPort(Device d3dDevice, List<Player> players)
        {

            float elapsedTime = GuiController.Instance.ElapsedTime;
                
            d3dDevice.Viewport = _leftViewPort;
            // Now we can clear just view-port's portion of the buffer to green...
            d3dDevice.Clear(ClearFlags.Target | ClearFlags.ZBuffer,
                        Color.FromArgb(255, 0, 255, 0), 1.0f, 0);

            d3dDevice.BeginScene();
            players[0].Cam.Enable = true;
            players[1].Cam.Enable = false;
            players[0].RenderPlayer(elapsedTime);
            players[1].RenderPlayer(elapsedTime);

            d3dDevice.EndScene();

            //LightAndReflection();
            /*
            
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

        private static Viewport LeftViewPortCreate()
        {
            var leftViewPort = new Viewport();
            leftViewPort.X = 0;
            leftViewPort.Y = 0;
            leftViewPort.Width = _originalWidth / 2;
            leftViewPort.Height = _originalHeight;
            leftViewPort.MinZ = 0.0f;
            leftViewPort.MaxZ = 1.0f;
            return leftViewPort;
        }


        private static Viewport RightViewPortCreate()
        {
            var rightViewPort = new Viewport();
            rightViewPort.X = _originalWidth / 2;
            rightViewPort.Y = 0;
            rightViewPort.Width = _originalWidth / 2;
            rightViewPort.Height = _originalHeight;
            rightViewPort.MinZ = 0.0f;
            rightViewPort.MaxZ = 1.0f;
            return rightViewPort;
        }


        private static int _originalWidth;
        private static int _originalHeight;
        private static Viewport _rightViewPort;
        private static Viewport _leftViewPort;
    }
}
