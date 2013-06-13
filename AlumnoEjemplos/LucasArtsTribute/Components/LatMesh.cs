using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;
using TgcViewer;
using TgcViewer.Utils;
using TgcViewer.Utils.TgcGeometry;
using TgcViewer.Utils.TgcSceneLoader;

namespace AlumnoEjemplos.LucasArtsTribute.Components
{
    public class LatMesh : TgcMesh
    {

        protected bool meshUpdated = true;

        public bool MeshUpdated
        {
            get { return meshUpdated; }
            set { meshUpdated = value; }
        }

        public Vector3 TransformCoord(Vector3 src)
        {
            if (!MeshUpdated)
                updateMeshTransform();

            src.TransformCoordinate(transform);
            return src;
        }

        public Vector3 Position
        {
            get { return translation; }
            set
            {
                MeshUpdated = false;
                translation = value;
                updateBoundingBox();
            }
        }

        public Vector3 Rotation
        {
            get { return rotation; }
            set 
            {
                MeshUpdated = false;
                rotation = value;
            }
        }

        public void SetRotationX(float x)
        {
            rotation.X = x;
        }

        public void SetRotationY(float y)
        {
            rotation.Y = y;
        }

        public void SetRotationZ(float z)
        {
            rotation.Z = z;
        }
        public Vector3 Scale
        {
            get { return scale; }
            set
            {
                MeshUpdated = false;
                scale = value;
                updateBoundingBox();
            }
        }
    }
}
