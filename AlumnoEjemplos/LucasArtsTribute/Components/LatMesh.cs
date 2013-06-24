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

        public static LATMesh FromTgcMesh(TgcMesh tgc)
        {
            LATMesh mesh = new LATMesh();

            mesh.d3dMesh = tgc.D3dMesh;
            mesh.name = tgc.Name;
            mesh.layer = tgc.Layer;
            mesh.UserProperties = tgc.UserProperties;
            mesh.materials = tgc.Materials;
            mesh.effect = tgc.Effect;
            mesh.technique = tgc.Technique;
            mesh.diffuseMaps = tgc.DiffuseMaps;
            mesh.lightMap = tgc.LightMap;
            mesh.enabled = tgc.Enabled;
            mesh.transform = tgc.Transform;
            mesh.autoTransformEnable = tgc.AutoTransformEnable;
            mesh.translation = tgc.Position;
            mesh.rotation = tgc.Rotation;
            mesh.scale = tgc.Scale;
            mesh.boundingBox = tgc.BoundingBox;
            mesh.renderType = tgc.RenderType;
            mesh.vertexDeclaration = tgc.VertexDeclaration;
            mesh.AutoUpdateBoundingBox = tgc.AutoUpdateBoundingBox;
            mesh.parentInstance = tgc.ParentInstance;
            mesh.meshInstances = tgc.MeshInstances;
            mesh.alphaBlendEnable = tgc.AlphaBlendEnable;

            return mesh;
        }

        protected LATMesh()
        {
        }

        public LATMesh(Mesh mesh, string name, MeshRenderType renderType) : base(mesh, name, renderType)
        {
            this.createBoundingBox();
        }
        /*
        public LATMesh(string name, TgcMesh parentInstance, Vector3 translation, Vector3 rotation, Vector3 scale) : base(name, parentInstance, translation, rotation, scale)
        {

        }
        */
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
