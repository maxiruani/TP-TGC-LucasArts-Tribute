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
using TgcViewer.Utils.Terrain;
using TgcViewer.Utils.TgcGeometry;

namespace AlumnoEjemplos.LucasArtsTribute
{
    class Multiplayer
    {
        public static List<Player> CreateMultiplayer(TgcBox piso, String configurationCar1, String configurationCar2, TgcSkyBox skyBox)
        {
            _skyBox = skyBox;
            _originalWidth = GuiController.Instance.D3dDevice.Viewport.Width;
            _originalHeight = GuiController.Instance.D3dDevice.Viewport.Height;
            List<Player> players = new List<Player>();

            Vector3 initialPosition = new Vector3(100, -50, 0);
            Player player = new Player(piso, configurationCar1, initialPosition, 1);
            players.Add(player);
            _downViewPort = RightViewPortCreate();

            initialPosition = new Vector3(10, -50, 0);
            player = new Player(piso, configurationCar2, initialPosition, 2);
            players.Add(player);
            _upViewPort = LeftViewPortCreate();

            GuiController.Instance.CustomRenderEnabled = true;


            return players;
        }

        public static void RenderAll(List<Player> players)
        {
            float elapsedTime = GuiController.Instance.ElapsedTime;
            players[0].DoPhysics(elapsedTime);
            players[1].DoPhysics(elapsedTime);

            int a = 0;

            /*
            if (Collision.TestOBB_Vs_OBB(players[0].Car.Obb, players[1].Car.Obb))
            {
                
            }
            */

            DownViewPort(GuiController.Instance.D3dDevice, players);
            //_skyBox.render();

            GuiController.Instance.CurrentCamera.updateCamera();
            GuiController.Instance.CurrentCamera.updateViewMatrix(GuiController.Instance.D3dDevice);

            UpViewPort(GuiController.Instance.D3dDevice, players);

            GuiController.Instance.D3dDevice.Present();

        }


        private static void DownViewPort(Device d3dDevice, List<Player> players)
        {
            // Set up view-port properties
            float elapsedTime = GuiController.Instance.ElapsedTime;

            d3dDevice.Viewport = _downViewPort;

            // Now we can clear just view-port's portion of the buffer to green...
            d3dDevice.Clear(ClearFlags.Target | ClearFlags.ZBuffer,
                            Color.FromArgb(255, 0, 255, 0), 1.0f, 0);

            d3dDevice.BeginScene();
            players[0].Cam.Enable = false;
            players[1].Cam.Enable = true;
            players[0].RenderPlayer(elapsedTime);
            players[1].RenderPlayer(elapsedTime);
            _skyBox.render();

            d3dDevice.EndScene();
        }

        private static void UpViewPort(Device d3dDevice, List<Player> players)
        {
            float elapsedTime = GuiController.Instance.ElapsedTime;
                
            d3dDevice.Viewport = _upViewPort;
            // Now we can clear just view-port's portion of the buffer to green...
            d3dDevice.Clear(ClearFlags.Target | ClearFlags.ZBuffer,
                        Color.FromArgb(255, 0, 255, 0), 1.0f, 0);

            d3dDevice.BeginScene();
            players[0].Cam.Enable = true;
            players[1].Cam.Enable = false;
            players[0].RenderPlayer(elapsedTime);
            players[1].RenderPlayer(elapsedTime);
            _skyBox.render();
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
            leftViewPort.Width = _originalWidth;
            leftViewPort.Height = _originalHeight / 2;
            leftViewPort.MinZ = 0.0f;
            leftViewPort.MaxZ = 1.0f;
            return leftViewPort;
        }


        private static Viewport RightViewPortCreate()
        {
            var rightViewPort = new Viewport();
            rightViewPort.X = 0;
            rightViewPort.Y = _originalHeight / 2;
            rightViewPort.Width = _originalWidth;
            rightViewPort.Height = _originalHeight / 2;
            rightViewPort.MinZ = 0.0f;
            rightViewPort.MaxZ = 1.0f;
            return rightViewPort;
        }


        private static int _originalWidth;
        private static int _originalHeight;
        private static Viewport _downViewPort;
        private static Viewport _upViewPort;
        private static TgcSkyBox _skyBox;
    }
}
