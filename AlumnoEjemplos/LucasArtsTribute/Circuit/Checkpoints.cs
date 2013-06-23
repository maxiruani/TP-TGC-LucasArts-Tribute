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
    public class Checkpoints
    {
        public Checkpoints(Vector3 center, Vector3 size, TgcTexture texture)
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
        public virtual void Render()
        {
            _box.render();
        }
        private Vector3 position;

        public void Dispose()
        {
            _box.dispose();
        }
    }

    public class NosBottle : Checkpoints
    {
        public NosBottle(Microsoft.DirectX.Direct3D.Device d3dDevice, Vector3 center, Vector3 size)
            : base(center, size, TgcTexture.createTexture(d3dDevice,
                                     GuiController.Instance.AlumnoEjemplosMediaDir + "LucasArtsTribute\\NosBottle.jpg"))
        {
            Obb = new OrientedBoundingBox(this.ObstacleBox.Position, this.ObstacleBox.Size);
        }

        public override void Render()
        {
            this.ObstacleBox.render();
            Obb.Render();
        }
        public OrientedBoundingBox Obb { get; internal set; }

    }
}
