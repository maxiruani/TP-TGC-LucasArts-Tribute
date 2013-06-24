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

    public class Checkpoint
    {
        public static List<NosBottle> CreateAllCheckPoints()
        {
            Random random = new Random(12);
            Vector3 nosBottleSize = new Vector3(5, 10, 5);
            List<NosBottle> checkpoints = new List<NosBottle>();
            for (int i = 0; i < 10; i++)
            {
                Vector3 randomPosition = new Vector3();
                randomPosition.X = random.Next(10, 2000);
                randomPosition.Z = random.Next(10, 2000);
                randomPosition.Y = -50;
                NosBottle checkpoint = new NosBottle(GuiController.Instance.D3dDevice, randomPosition, nosBottleSize);
                checkpoints.Add(checkpoint);
            }
            return checkpoints;
        } 

        public Checkpoint(Vector3 center, Vector3 size, TgcTexture texture)
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

    public class NosBottle : Checkpoint
    {
        public NosBottle(Microsoft.DirectX.Direct3D.Device d3dDevice, Vector3 center, Vector3 size)
            : base(center, size, TgcTexture.createTexture(d3dDevice,
                                     GuiController.Instance.AlumnoEjemplosMediaDir + "LucasArtsTribute\\NosBottle.jpg"))
        {
            Obb = OrientedBoundingBox.computeFromAABB(TgcBox.fromSize(center, size).BoundingBox);
            Obb.Center = this.ObstacleBox.Position;
            Enable = true;
        }

        public override void Render()
        {
            if (Enable)
            {
                this.ObstacleBox.render();
                Obb.render();
            }
        }
        public OrientedBoundingBox Obb { get; internal set; }

        public bool Enable { get; set; }

    }
}
