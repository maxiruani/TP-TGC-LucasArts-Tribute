using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using Microsoft.DirectX;
using TgcViewer;
using TgcViewer.Utils.TgcGeometry;
using TgcViewer.Utils.TgcSceneLoader;

namespace AlumnoEjemplos.LucasArtsTribute.Circuit
{
    public class Obstacle
    {
        public Obstacle(Vector3 center, Vector3 size, TgcTexture texture)
        {
            _box = TgcBox.fromSize(center, size, texture);
        }

        private TgcBox _box;
        public TgcBox ObstacleBox 
        { 
            get { return _box; }
        }

        public Vector3 Position
        {
            get { return _box.Position; }
        }
        public void Render()
        {
            _box.render();
        }
        private Vector3 position;

        public void Dispose()
        {
            _box.dispose();
        }
    }

    public class Wheel : Obstacle
    {
        public Wheel(Microsoft.DirectX.Direct3D.Device d3dDevice, Vector3 center, Vector3 size)
            : base(center, size, TgcTexture.createTexture(d3dDevice,
                                     GuiController.Instance.ExamplesMediaDir + "Texturas\\Quake\\TexturePack3\\goo2.jpg"))
        {
        }

    }
}
