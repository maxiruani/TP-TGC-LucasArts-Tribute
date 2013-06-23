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
    public class LATMesh : TgcMesh
    {

        public Vector3 TransformCoord(Vector3 src)
        {
            Matrix transf = Matrix.RotationYawPitchRoll(rotation.Y, rotation.X, rotation.Z) * Matrix.Translation(translation);

            src.TransformCoordinate(transf);
            return src;
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

    }
}
