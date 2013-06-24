using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using AlumnoEjemplos.LucasArtsTribute.VehicleModel;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;
using TgcViewer;
using TgcViewer.Utils;
using TgcViewer.Utils.TgcGeometry;
using TgcViewer.Utils.TgcSceneLoader;

namespace AlumnoEjemplos.LucasArtsTribute
{
    public class OrientedBoundingBox
    {
        public OrientedBoundingBox()
        {
           // _renderColor = Color.Yellow.ToArgb();
            _dirtyValues = true;
            AlphaBlendEnable = false;
        }


        public OrientedBoundingBox(Vehicle car)// Vector3 centro, Vector3 axisX, Vector3 axisY, Vector3 axisZ, float[] mediaLongAxis)
            : this()
        {
            _car = car;

            _axis[0] = new Vector3(1, 0, 0);
            _axis[1] = new Vector3(0, 1, 0);
            _axis[2] = new Vector3(0, 0, 1);
            _center = car.body.BoundingBox.calculateBoxCenter();

            _originalCenter = car.body.BoundingBox.calculateBoxCenter();

            Vector3 radios = car.body.BoundingBox.calculateAxisRadius();
            float[] mediaLongitud = new float[] { radios.X, radios.Y, radios.Z };

            _mediaLongitudAxis = mediaLongitud;
            _originalMediaLongitudAxis[0] = mediaLongitud[0];
            _originalMediaLongitudAxis[1] = mediaLongitud[1];
            _originalMediaLongitudAxis[2] = mediaLongitud[2];
        }

        public OrientedBoundingBox(Vector3 center, Vector3 size)// Vector3 centro, Vector3 axisX, Vector3 axisY, Vector3 axisZ, float[] mediaLongAxis)
            : this()
        {

            _axis[0] = new Vector3(1, 0, 0);
            _axis[1] = new Vector3(0, 1, 0);
            _axis[2] = new Vector3(0, 0, 1);
            _center = center;

            _originalCenter = center;

            Vector3 radios = size;
            float[] mediaLongitud = new float[] { radios.X, radios.Y, radios.Z };

            _mediaLongitudAxis = mediaLongitud;
            _originalMediaLongitudAxis[0] = mediaLongitud[0];
            _originalMediaLongitudAxis[1] = mediaLongitud[1];
            _originalMediaLongitudAxis[2] = mediaLongitud[2];
        }

        /// <summary>
        ///     Habilita el renderizado con AlphaBlending para los modelos
        ///     con textura o colores por vértice de canal Alpha.
        ///     Por default está deshabilitado.
        /// </summary>
        public bool AlphaBlendEnable { get; set; }


        public void setExtremes(Vector3 punto1, Vector3 punto2, Vector3 punto3, Vector3 punto4)
        {
            _dirtyValues = true;
        }


        /// <summary>
        ///     Actualizar los valores de los vertices a renderizar
        /// </summary>
        private void updateValues()
        {
            if (_vertices == null)
            {
                _vertices = _vertices = new CustomVertex.PositionColored[24];
            }

            //Cuadrado de atras


            _vertices[0] =
                new CustomVertex.PositionColored(
                    Center + MediaLongitudAxis[0] * Axis[0] + MediaLongitudAxis[1] * Axis[1] + MediaLongitudAxis[2] * Axis[2],
                    _renderColor);
            _vertices[1] =
                new CustomVertex.PositionColored(
                    Center + MediaLongitudAxis[0] * Axis[0] - MediaLongitudAxis[1] * Axis[1] + MediaLongitudAxis[2] * Axis[2],
                    _renderColor);

            _vertices[2] =
                new CustomVertex.PositionColored(
                    Center + MediaLongitudAxis[0] * Axis[0] - MediaLongitudAxis[1] * Axis[1] + MediaLongitudAxis[2] * Axis[2],
                    _renderColor);
            _vertices[3] =
                new CustomVertex.PositionColored(
                    Center - MediaLongitudAxis[0] * Axis[0] - MediaLongitudAxis[1] * Axis[1] + MediaLongitudAxis[2] * Axis[2],
                    _renderColor);

            _vertices[4] =
                new CustomVertex.PositionColored(
                    Center - MediaLongitudAxis[0] * Axis[0] - MediaLongitudAxis[1] * Axis[1] + MediaLongitudAxis[2] * Axis[2],
                    _renderColor);
            _vertices[5] =
                new CustomVertex.PositionColored(
                    Center - MediaLongitudAxis[0] * Axis[0] + MediaLongitudAxis[1] * Axis[1] + MediaLongitudAxis[2] * Axis[2],
                    _renderColor);

            _vertices[6] =
                new CustomVertex.PositionColored(
                    Center - MediaLongitudAxis[0] * Axis[0] + MediaLongitudAxis[1] * Axis[1] + MediaLongitudAxis[2] * Axis[2],
                    _renderColor);
            _vertices[7] =
                new CustomVertex.PositionColored(
                    Center + MediaLongitudAxis[0] * Axis[0] + MediaLongitudAxis[1] * Axis[1] + MediaLongitudAxis[2] * Axis[2],
                    _renderColor);

            //Cuadrado de adelante
            _vertices[8] =
                new CustomVertex.PositionColored(
                    Center + MediaLongitudAxis[0] * Axis[0] + MediaLongitudAxis[1] * Axis[1] - MediaLongitudAxis[2] * Axis[2],
                    _renderColor);
            _vertices[9] =
                new CustomVertex.PositionColored(
                    Center + MediaLongitudAxis[0] * Axis[0] - MediaLongitudAxis[1] * Axis[1] - MediaLongitudAxis[2] * Axis[2],
                    _renderColor);

            _vertices[10] =
                new CustomVertex.PositionColored(
                    Center + MediaLongitudAxis[0] * Axis[0] - MediaLongitudAxis[1] * Axis[1] - MediaLongitudAxis[2] * Axis[2],
                    _renderColor);
            _vertices[11] =
                new CustomVertex.PositionColored(
                    Center - MediaLongitudAxis[0] * Axis[0] - MediaLongitudAxis[1] * Axis[1] - MediaLongitudAxis[2] * Axis[2],
                    _renderColor);

            _vertices[12] =
                new CustomVertex.PositionColored(
                    Center - MediaLongitudAxis[0] * Axis[0] - MediaLongitudAxis[1] * Axis[1] - MediaLongitudAxis[2] * Axis[2],
                    _renderColor);
            _vertices[13] =
                new CustomVertex.PositionColored(
                    Center - MediaLongitudAxis[0] * Axis[0] + MediaLongitudAxis[1] * Axis[1] - MediaLongitudAxis[2] * Axis[2],
                    _renderColor);

            _vertices[14] =
                new CustomVertex.PositionColored(
                    Center - MediaLongitudAxis[0] * Axis[0] + MediaLongitudAxis[1] * Axis[1] - MediaLongitudAxis[2] * Axis[2],
                    _renderColor);
            _vertices[15] =
                new CustomVertex.PositionColored(
                    Center + MediaLongitudAxis[0] * Axis[0] + MediaLongitudAxis[1] * Axis[1] - MediaLongitudAxis[2] * Axis[2],
                    _renderColor);

            //Union de ambos cuadrados
            _vertices[16] =
                new CustomVertex.PositionColored(
                    Center + MediaLongitudAxis[0] * Axis[0] + MediaLongitudAxis[1] * Axis[1] + MediaLongitudAxis[2] * Axis[2],
                    _renderColor);
            _vertices[17] =
                new CustomVertex.PositionColored(
                    Center + MediaLongitudAxis[0] * Axis[0] + MediaLongitudAxis[1] * Axis[1] - MediaLongitudAxis[2] * Axis[2],
                    _renderColor);

            _vertices[18] =
                new CustomVertex.PositionColored(
                    Center + MediaLongitudAxis[0] * Axis[0] - MediaLongitudAxis[1] * Axis[1] + MediaLongitudAxis[2] * Axis[2],
                    _renderColor);
            _vertices[19] =
                new CustomVertex.PositionColored(
                    Center + MediaLongitudAxis[0] * Axis[0] - MediaLongitudAxis[1] * Axis[1] - MediaLongitudAxis[2] * Axis[2],
                    _renderColor);

            _vertices[20] =
                new CustomVertex.PositionColored(
                    Center - MediaLongitudAxis[0] * Axis[0] + MediaLongitudAxis[1] * Axis[1] + MediaLongitudAxis[2] * Axis[2],
                    _renderColor);
            _vertices[21] =
                new CustomVertex.PositionColored(
                    Center - MediaLongitudAxis[0] * Axis[0] + MediaLongitudAxis[1] * Axis[1] - MediaLongitudAxis[2] * Axis[2],
                    _renderColor);

            _vertices[22] =
                new CustomVertex.PositionColored(
                    Center - MediaLongitudAxis[0] * Axis[0] - MediaLongitudAxis[1] * Axis[1] + MediaLongitudAxis[2] * Axis[2],
                    _renderColor);
            _vertices[23] =
                new CustomVertex.PositionColored(
                    Center - MediaLongitudAxis[0] * Axis[0] - MediaLongitudAxis[1] * Axis[1] - MediaLongitudAxis[2] * Axis[2],
                    _renderColor);
        }


        /// <summary>
        ///     Libera los recursos del objeto
        /// </summary>
        public void dispose()
        {
            _vertices = null;
        }


        /// <summary>
        ///     Renderizar BoundingBox
        /// </summary>
        public void Render()
        {
            Device d3dDevice = GuiController.Instance.D3dDevice;
            TgcTexture.Manager texturesManager = GuiController.Instance.TexturesManager;

            texturesManager.clear(0);
            texturesManager.clear(1);
            d3dDevice.Material = TgcD3dDevice.DEFAULT_MATERIAL;
            d3dDevice.Transform.World = Matrix.Identity;

            //Actualizar vertices de BoundingBox solo si hubo una modificación
            if (_dirtyValues)
            {
                updateValues();
                _dirtyValues = false;
            }

            d3dDevice.VertexFormat = CustomVertex.PositionColored.Format;
            d3dDevice.DrawUserPrimitives(PrimitiveType.LineList, 12, _vertices);
        }


        public void Move()
        {
            _center = _car.body.BoundingBox.calculateBoxCenter();
            _dirtyValues = true;
            _angle = _car.SOmega;
            Rotar();
        }


        private float _angle;
        private void Rotar()
        {
            float angle = Geometry.DegreeToRadian(_angle);
            Vector3 rotation = new Vector3(0, angle , 0);
            Vector3[] newAxis = new Vector3[3];
            
            Matrix rotMat = Matrix.RotationYawPitchRoll(-rotation.Y, -rotation.X, -rotation.Z);

            Matrix axisActual = new Matrix();

            for (int i = 0; i <= 2; i++)
            {
                axisActual.M11 = _axis[i].X;
                axisActual.M21 = _axis[i].Y;
                axisActual.M31 = _axis[i].Z;
                axisActual.M41 = 0;
                
                axisActual = rotMat * axisActual;

                newAxis[i].X = axisActual.M11;
                newAxis[i].Y = axisActual.M21;
                newAxis[i].Z = axisActual.M31;
            }

            for (int i = 0; i <= 2; i++)
                _axis[i] = newAxis[i];

            _dirtyValues = true;
        }


        /// <summary>
        ///     Crea un array con los 8 vertices del OrientedBoundingBox, en base a los extremos especificados
        /// </summary>
        private Vector3[] ComputeCorners(Vector3 centro)
        {
            var corners = new Vector3[8];

            corners[0] = centro + MediaLongitudAxis[0] * Axis[0] + MediaLongitudAxis[1] * Axis[1] +
                         MediaLongitudAxis[2] * Axis[2];
            corners[1] = centro + MediaLongitudAxis[0] * Axis[0] - MediaLongitudAxis[1] * Axis[1] +
                         MediaLongitudAxis[2] * Axis[2];

            corners[2] = centro - MediaLongitudAxis[0] * Axis[0] - MediaLongitudAxis[1] * Axis[1] +
                         MediaLongitudAxis[2] * Axis[2];
            corners[3] = centro - MediaLongitudAxis[0] * Axis[0] + MediaLongitudAxis[1] * Axis[1] +
                         MediaLongitudAxis[2] * Axis[2];

            corners[4] = centro + MediaLongitudAxis[0] * Axis[0] + MediaLongitudAxis[1] * Axis[1] -
                         MediaLongitudAxis[2] * Axis[2];
            corners[5] = centro + MediaLongitudAxis[0] * Axis[0] - MediaLongitudAxis[1] * Axis[1] -
                         MediaLongitudAxis[2] * Axis[2];

            corners[6] = centro - MediaLongitudAxis[0] * Axis[0] - MediaLongitudAxis[1] * Axis[1] -
                         MediaLongitudAxis[2] * Axis[2];
            corners[7] = centro - MediaLongitudAxis[0] * Axis[0] + MediaLongitudAxis[1] * Axis[1] -
                         MediaLongitudAxis[2] * Axis[2];

            return corners;
        }

        /// <summary>
        ///     Crea un array con los 8 vertices del OrientedBoundingBox
        /// </summary>
        public Vector3[] ComputeCorners()
        {
            return ComputeCorners(_center);
        }


        /// <summary>
        ///     Calcula los polígonos que conforman las 6 caras del BoundingBox
        /// </summary>
        /// <returns>Array con las 6 caras del polígono en el siguiente orden: Up, Down, Front, Back, Right, Left</returns>
        public FaceOBB[] computeFaces()
        {
            var faces = new FaceOBB[6];
            FaceOBB face;

            Vector3[] vertices = ComputeCorners();


            //Up
            face = new FaceOBB();
            face.Plane = Plane.FromPoints(vertices[0], vertices[3], vertices[4]);
            face.Extremes = new[]
                {
                    new Vector3(vertices[0].X, vertices[0].Y, vertices[0].Z),
                    new Vector3(vertices[3].X, vertices[3].Y, vertices[3].Z),
                    new Vector3(vertices[4].X, vertices[4].Y, vertices[4].Z),
                    new Vector3(vertices[7].X, vertices[7].Y, vertices[7].Z)
                };
            faces[0] = face;

            //Down
            face = new FaceOBB();
            face.Plane = Plane.FromPoints(vertices[1], vertices[2], vertices[5]);
            face.Extremes = new[]
                {
                    new Vector3(vertices[1].X, vertices[1].Y, vertices[1].Z),
                    new Vector3(vertices[2].X, vertices[2].Y, vertices[2].Z),
                    new Vector3(vertices[5].X, vertices[5].Y, vertices[5].Z),
                    new Vector3(vertices[6].X, vertices[6].Y, vertices[6].Z)
                };
            faces[1] = face;

            //Front
            face = new FaceOBB();
            face.Plane = Plane.FromPoints(vertices[4], vertices[5], vertices[6]);
            face.Extremes = new[]
                {
                    new Vector3(vertices[4].X, vertices[4].Y, vertices[4].Z),
                    new Vector3(vertices[5].X, vertices[5].Y, vertices[5].Z),
                    new Vector3(vertices[6].X, vertices[6].Y, vertices[6].Z),
                    new Vector3(vertices[7].X, vertices[7].Y, vertices[7].Z)
                };
            faces[2] = face;

            //Back
            face = new FaceOBB();
            face.Plane = Plane.FromPoints(vertices[0], vertices[1], vertices[2]);
            face.Extremes = new[]
                {
                    new Vector3(vertices[0].X, vertices[0].Y, vertices[0].Z),
                    new Vector3(vertices[1].X, vertices[1].Y, vertices[1].Z),
                    new Vector3(vertices[2].X, vertices[2].Y, vertices[2].Z),
                    new Vector3(vertices[3].X, vertices[3].Y, vertices[3].Z)
                };
            faces[3] = face;

            //Right
            face = new FaceOBB();
            face.Plane = Plane.FromPoints(vertices[0], vertices[1], vertices[5]);
            face.Extremes = new[]
                {
                    new Vector3(vertices[0].X, vertices[0].Y, vertices[0].Z),
                    new Vector3(vertices[1].X, vertices[1].Y, vertices[1].Z),
                    new Vector3(vertices[5].X, vertices[5].Y, vertices[5].Z),
                    new Vector3(vertices[4].X, vertices[4].Y, vertices[4].Z)
                };
            faces[4] = face;

            //Left
            face = new FaceOBB();
            face.Plane = Plane.FromPoints(vertices[2], vertices[3], vertices[7]);
            face.Extremes = new[]
                {
                    new Vector3(vertices[2].X, vertices[2].Y, vertices[2].Z),
                    new Vector3(vertices[3].X, vertices[3].Y, vertices[3].Z),
                    new Vector3(vertices[7].X, vertices[7].Y, vertices[7].Z),
                    new Vector3(vertices[6].X, vertices[6].Y, vertices[6].Z)
                };
            faces[5] = face;

            return faces;
        }


        public OrientedBoundingBox actualizarDesdeAABB(TgcBoundingBox bb)
        {
            _center = bb.calculateBoxCenter();

            _axis[0] = new Vector3(1, 0, 0);
            _axis[1] = new Vector3(0, 1, 0);
            _axis[2] = new Vector3(0, 0, 1);

            Vector3 radios = bb.calculateAxisRadius();

            _mediaLongitudAxis[0] = radios.X;
            _mediaLongitudAxis[1] = radios.Y;
            _mediaLongitudAxis[2] = radios.Z;

            return this;
        }

        /// <summary>
        ///     Cara de un OrientedBoundingBox representada por un polígono rectangular de 4 vértices.
        /// </summary>
        public class FaceOBB
        {
            public FaceOBB()
            {
                Extremes = new Vector3[4];
            }

            /// <summary>
            ///     Los 4 vértices extremos de la cara
            /// </summary>
            public Vector3[] Extremes { get; set; }

            /// <summary>
            ///     Ecuación del plano que engloba la cara, con su normal apuntado hacia afuera normalizada.
            /// </summary>
            public Plane Plane { get; set; }
        }


        public Vector3 Center
        {
            get { return _center; }
            set { _center = value; }
        }

        public Vector3[] Axis
        {
            get { return _axis; }
            set { _axis = value; }
        }

        public float[] MediaLongitudAxis
        {
            get { return _mediaLongitudAxis; }
            set { _mediaLongitudAxis = value; }
        }

        public Vector3 OriginalCenter
        {
            get { return _originalCenter; }
        }

        /// <summary>
        ///     Color de renderizado del BoundingBox.
        /// </summary>
        public int RenderColor
        {
            get { return _renderColor; }
        }

        private readonly Vector3 _originalCenter = Vector3.Empty;

        private readonly float[] _originalMediaLongitudAxis = new float[3];


        private readonly int _renderColor;
        private Vector3[] _axis = new Vector3[3];
        private Vector3 _center = Vector3.Empty;

        private bool _dirtyValues;
        private float[] _mediaLongitudAxis = new float[3];
        private CustomVertex.PositionColored[] _vertices;
        private Vehicle _car;
    }
    
}
