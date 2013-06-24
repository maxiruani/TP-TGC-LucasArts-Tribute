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
using TgcViewer.Utils.Shaders;
using TgcViewer.Utils.TgcGeometry;
using TgcViewer.Utils.TgcSceneLoader;

namespace AlumnoEjemplos.LucasArtsTribute
{
    public class OrientedBoundingBox
    {
        private Vector3 center;

        /// <summary>
        /// Centro
        /// </summary>
        public Vector3 Center
        {
            get { return center; }
            set
            {
                center = value;
                dirtyValues = true;
            }
        }

        private Vector3[] orientation = new Vector3[3];

        /// <summary>
        /// Orientacion del OBB, expresada en local axes
        /// </summary>
        public Vector3[] Orientation
        {
            get { return orientation; }
            set
            {
                orientation = value;
                dirtyValues = true;
            }
        }

        private Vector3 extents;

        /// <summary>
        /// Radios
        /// </summary>
        public Vector3 Extents
        {
            get { return extents; }
            set
            {
                extents = value;
                dirtyValues = true;
            }
        }


        private int renderColor;

        /// <summary>
        /// Color de renderizado del BoundingBox.
        /// </summary>
        public int RenderColor
        {
            get { return renderColor; }
        }

        public Vector3 Position
        {
            get { return center; }
        }

        private bool alphaBlendEnable;

        /// <summary>
        /// Habilita el renderizado con AlphaBlending para los modelos
        /// con textura o colores por vértice de canal Alpha.
        /// Por default está deshabilitado.
        /// </summary>
        public bool AlphaBlendEnable
        {
            get { return alphaBlendEnable; }
            set { alphaBlendEnable = value; }
        }

        protected Effect effect;

        /// <summary>
        /// Shader del mesh
        /// </summary>
        public Effect Effect
        {
            get { return effect; }
            set { effect = value; }
        }

        protected string technique;

        /// <summary>
        /// Technique que se va a utilizar en el effect.
        /// Cada vez que se llama a render() se carga este Technique (pisando lo que el shader ya tenia seteado)
        /// </summary>
        public string Technique
        {
            get { return technique; }
            set { technique = value; }
        }


        private CustomVertex.PositionColored[] vertices;
        private bool dirtyValues;

        /// <summary>
        /// Construir OBB vacio
        /// </summary>
        public OrientedBoundingBox()
        {
            renderColor = Color.Yellow.ToArgb();
            dirtyValues = true;
            alphaBlendEnable = false;
        }





        /// <summary>
        /// Configurar el color de renderizado del OBB
        /// Ejemplo: Color.Yellow.ToArgb();
        /// </summary>
        public void setRenderColor(Color color)
        {
            this.renderColor = color.ToArgb();
            dirtyValues = true;
        }

        /// <summary>
        /// Actualizar los valores de los vertices a renderizar
        /// </summary>
        public void updateValues()
        {
            if (vertices == null)
            {
                vertices = vertices = new CustomVertex.PositionColored[24];
            }

            Vector3[] corners = computeCorners();


            //Cuadrado de atras
            vertices[0] = new CustomVertex.PositionColored(corners[0], renderColor);
            vertices[1] = new CustomVertex.PositionColored(corners[4], renderColor);

            vertices[2] = new CustomVertex.PositionColored(corners[0], renderColor);
            vertices[3] = new CustomVertex.PositionColored(corners[2], renderColor);

            vertices[4] = new CustomVertex.PositionColored(corners[2], renderColor);
            vertices[5] = new CustomVertex.PositionColored(corners[6], renderColor);

            vertices[6] = new CustomVertex.PositionColored(corners[4], renderColor);
            vertices[7] = new CustomVertex.PositionColored(corners[6], renderColor);

            //Cuadrado de adelante
            vertices[8] = new CustomVertex.PositionColored(corners[1], renderColor);
            vertices[9] = new CustomVertex.PositionColored(corners[5], renderColor);

            vertices[10] = new CustomVertex.PositionColored(corners[1], renderColor);
            vertices[11] = new CustomVertex.PositionColored(corners[3], renderColor);

            vertices[12] = new CustomVertex.PositionColored(corners[3], renderColor);
            vertices[13] = new CustomVertex.PositionColored(corners[7], renderColor);

            vertices[14] = new CustomVertex.PositionColored(corners[5], renderColor);
            vertices[15] = new CustomVertex.PositionColored(corners[7], renderColor);

            //Union de ambos cuadrados
            vertices[16] = new CustomVertex.PositionColored(corners[0], renderColor);
            vertices[17] = new CustomVertex.PositionColored(corners[1], renderColor);

            vertices[18] = new CustomVertex.PositionColored(corners[4], renderColor);
            vertices[19] = new CustomVertex.PositionColored(corners[5], renderColor);

            vertices[20] = new CustomVertex.PositionColored(corners[2], renderColor);
            vertices[21] = new CustomVertex.PositionColored(corners[3], renderColor);

            vertices[22] = new CustomVertex.PositionColored(corners[6], renderColor);
            vertices[23] = new CustomVertex.PositionColored(corners[7], renderColor);
        }

        /// <summary>
        /// Crea un array con los 8 vertices del OBB
        /// </summary>
        public Vector3[] computeCorners()
        {
            Vector3[] corners = new Vector3[8];

            Vector3 eX = extents.X*orientation[0];
            Vector3 eY = extents.Y*orientation[1];
            Vector3 eZ = extents.Z*orientation[2];

            corners[0] = center - eX - eY - eZ;
            corners[1] = center - eX - eY + eZ;

            corners[2] = center - eX + eY - eZ;
            corners[3] = center - eX + eY + eZ;

            corners[4] = center + eX - eY - eZ;
            corners[5] = center + eX - eY + eZ;

            corners[6] = center + eX + eY - eZ;
            corners[7] = center + eX + eY + eZ;

            return corners;
        }

        /// <summary>
        /// Renderizar
        /// </summary>
        public void render()
        {
            Device d3dDevice = GuiController.Instance.D3dDevice;
            TgcTexture.Manager texturesManager = GuiController.Instance.TexturesManager;

            texturesManager.clear(0);
            texturesManager.clear(1);

            //Cargar shader si es la primera vez
            if (this.effect == null)
            {
                this.effect = GuiController.Instance.Shaders.VariosShader;
                this.technique = TgcShaders.T_POSITION_COLORED;
            }

            //Actualizar vertices de BoundingBox solo si hubo una modificación
            if (dirtyValues)
            {
                updateValues();
                dirtyValues = false;
            }

            GuiController.Instance.Shaders.setShaderMatrixIdentity(this.effect);
            d3dDevice.VertexDeclaration = GuiController.Instance.Shaders.VdecPositionColored;
            effect.Technique = this.technique;

            //Render con shader
            effect.Begin(0);
            effect.BeginPass(0);
            d3dDevice.DrawUserPrimitives(PrimitiveType.LineList, 12, vertices);
            effect.EndPass();
            effect.End();
        }

        /// <summary>
        /// Libera los recursos del objeto
        /// </summary>
        public void dispose()
        {
            vertices = null;
        }

        /// <summary>
        /// Mueve el centro del OBB
        /// </summary>
        /// <param name="movement">Movimiento relativo que se quiere aplicar</param>
        public void move(Vector3 movement)
        {
            center += movement;
            dirtyValues = true;
        }

        /// <summary>
        /// Rotar OBB en los 3 ejes.
        /// Es una rotacion relativa, sumando a lo que ya tenia antes de rotacion.
        /// </summary>
        /// <param name="movement">Ángulo de rotación de cada eje en radianes</param>
        public void rotate(Vector3 rotation)
        {
            Matrix rotM = Matrix.RotationYawPitchRoll(rotation.Y, rotation.X, rotation.Z);
            Matrix currentRotM = computeRotationMatrix();
            Matrix newRotM = currentRotM*rotM;

            orientation[0] = new Vector3(newRotM.M11, newRotM.M12, newRotM.M13);
            orientation[1] = new Vector3(newRotM.M21, newRotM.M22, newRotM.M23);
            orientation[2] = new Vector3(newRotM.M31, newRotM.M32, newRotM.M33);

            dirtyValues = true;
        }

        /// <summary>
        /// Cargar la rotacion absoluta del OBB.
        /// Pierda la rotacion anterior.
        /// </summary>
        /// <param name="rotation">Ángulo de rotación de cada eje en radianes</param>
        public void setRotation(Vector3 rotation)
        {
            Matrix rotM = Matrix.RotationYawPitchRoll(rotation.Y, rotation.X, rotation.Z);
            orientation[0] = new Vector3(rotM.M11, rotM.M12, rotM.M13);
            orientation[1] = new Vector3(rotM.M21, rotM.M22, rotM.M23);
            orientation[2] = new Vector3(rotM.M31, rotM.M32, rotM.M33);

            dirtyValues = true;
        }

        /// <summary>
        /// Calcula la matriz de rotacion 4x4 del Obb en base a su orientacion
        /// </summary>
        /// <returns>Matriz de rotacion de 4x4</returns>
        public Matrix computeRotationMatrix()
        {
            Matrix rot = Matrix.Identity;

            rot.M11 = orientation[0].X;
            rot.M12 = orientation[0].Y;
            rot.M13 = orientation[0].Z;

            rot.M21 = orientation[1].X;
            rot.M22 = orientation[1].Y;
            rot.M23 = orientation[1].Z;

            rot.M31 = orientation[2].X;
            rot.M32 = orientation[2].Y;
            rot.M33 = orientation[2].Z;

            return rot;
        }

        /// <summary>
        /// Calcular OBB a partir de un conjunto de puntos.
        /// Busca por fuerza bruta el mejor OBB en la mejor orientación que se ajusta a esos puntos.
        /// Es un calculo costoso.
        /// </summary>
        /// <param name="points">puntos</param>
        /// <returns>OBB calculado</returns>
        public static OrientedBoundingBox computeFromPoints(Vector3[] points)
        {
            return computeFromPointsRecursive(points, new Vector3(0, 0, 0), new Vector3(360, 360, 360), 10f).toClass();
        }


        /// <summary>
        /// Calcular OBB a partir de un conjunto de puntos.
        /// Prueba todas las orientaciones entre initValues y endValues, saltando de angulo en cada intervalo segun step
        /// Continua recursivamente hasta llegar a un step menor a 0.01f
        /// </summary>
        /// <returns></returns>
        private static OBBStruct computeFromPointsRecursive(Vector3[] points, Vector3 initValues, Vector3 endValues,
                                                            float step)
        {
            OBBStruct minObb = new OBBStruct();
            float minVolume = float.MaxValue;
            Vector3 minInitValues = Vector3.Empty;
            Vector3 minEndValues = Vector3.Empty;
            Vector3[] transformedPoints = new Vector3[points.Length];
            float x, y, z;


            x = initValues.X;
            while (x <= endValues.X)
            {
                y = initValues.Y;
                float rotX = FastMath.ToRad(x);
                while (y <= endValues.Y)
                {
                    z = initValues.Z;
                    float rotY = FastMath.ToRad(y);
                    while (z <= endValues.Z)
                    {
                        //Matriz de rotacion
                        float rotZ = FastMath.ToRad(z);
                        Matrix rotM = Matrix.RotationYawPitchRoll(rotY, rotX, rotZ);
                        Vector3[] orientation = new Vector3[]
                            {
                                new Vector3(rotM.M11, rotM.M12, rotM.M13),
                                new Vector3(rotM.M21, rotM.M22, rotM.M23),
                                new Vector3(rotM.M31, rotM.M32, rotM.M33)
                            };

                        //Transformar todos los puntos a OBB-space
                        for (int i = 0; i < transformedPoints.Length; i++)
                        {
                            transformedPoints[i].X = Vector3.Dot(points[i], orientation[0]);
                            transformedPoints[i].Y = Vector3.Dot(points[i], orientation[1]);
                            transformedPoints[i].Z = Vector3.Dot(points[i], orientation[2]);
                        }

                        //Obtener el AABB de todos los puntos transformados
                        TgcBoundingBox aabb = TgcBoundingBox.computeFromPoints(transformedPoints);

                        //Calcular volumen del AABB
                        Vector3 extents = aabb.calculateAxisRadius();
                        extents = TgcVectorUtils.abs(extents);
                        float volume = extents.X*2*extents.Y*2*extents.Z*2;

                        //Buscar menor volumen
                        if (volume < minVolume)
                        {
                            minVolume = volume;
                            minInitValues = new Vector3(x, y, z);
                            minEndValues = new Vector3(x + step, y + step, z + step);

                            //Volver centro del AABB a World-space
                            Vector3 center = aabb.calculateBoxCenter();
                            center = center.X*orientation[0] + center.Y*orientation[1] + center.Z*orientation[2];

                            //Crear OBB
                            minObb.center = center;
                            minObb.extents = extents;
                            minObb.orientation = orientation;
                        }

                        z += step;
                    }
                    y += step;
                }
                x += step;
            }

            //Recursividad en mejor intervalo encontrado
            if (step > 0.01f)
            {
                minObb = computeFromPointsRecursive(points, minInitValues, minEndValues, step/10f);
            }

            return minObb;
        }

        /// <summary>
        /// Generar OBB a partir de AABB
        /// </summary>
        /// <param name="aabb">BoundingBox</param>
        /// <returns>OBB generado</returns>
        public static OrientedBoundingBox computeFromAABB(TgcBoundingBox aabb)
        {
            return computeFromAABB(aabb.toStruct()).toClass();
        }

        /// <summary>
        /// Generar OBB a partir de AABB
        /// </summary>
        /// <param name="aabb">BoundingBox</param>
        /// <returns>OBB generado</returns>
        public static OBBStruct computeFromAABB(TgcBoundingBox.AABBStruct aabb)
        {
            OBBStruct obb = new OBBStruct();
            obb.extents = (aabb.max - aabb.min)*0.5f;
            obb.center = aabb.min + obb.extents;

            obb.orientation = new Vector3[] {new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1)};
            return obb;
        }


        /// <summary>
        /// Convertir un punto de World-Space espacio de coordenadas del OBB (OBB-Space)
        /// </summary>
        /// <param name="p">Punto en World-space</param>
        /// <returns>Punto convertido a OBB-space</returns>
        public Vector3 toObbSpace(Vector3 p)
        {
            Vector3 t = p - center;
            return new Vector3(Vector3.Dot(t, orientation[0]), Vector3.Dot(t, orientation[1]),
                               Vector3.Dot(t, orientation[2]));
        }

        /// <summary>
        /// Convertir un punto de OBB-space a World-space
        /// </summary>
        /// <param name="p">Punto en OBB-space</param>
        /// <returns>Punto convertido a World-space</returns>
        public Vector3 toWorldSpace(Vector3 p)
        {
            return center + p.X*orientation[0] + p.Y*orientation[1] + p.Z*orientation[2];
        }



        /// <summary>
        /// Convertir a struct
        /// </summary>
        public TgcObb.OBBStruct toStruct()
        {
            TgcObb.OBBStruct obbStruct = new TgcObb.OBBStruct();
            obbStruct.center = center;
            obbStruct.orientation = orientation;
            obbStruct.extents = extents;
            return obbStruct;
        }

        /// <summary>
        /// OBB en un struct liviano
        /// </summary>
        public struct OBBStruct
        {
            public Vector3 center;
            public Vector3[] orientation;
            public Vector3 extents;

            /// <summary>
            /// Convertir a clase
            /// </summary>
            public OrientedBoundingBox toClass()
            {
                OrientedBoundingBox obb = new OrientedBoundingBox();
                obb.center = center;
                obb.orientation = orientation;
                obb.extents = extents;
                return obb;
            }

            /// <summary>
            /// Convertir un punto de World-Space espacio de coordenadas del OBB (OBB-Space)
            /// </summary>
            /// <param name="p">Punto en World-space</param>
            /// <returns>Punto convertido a OBB-space</returns>
            public Vector3 toObbSpace(Vector3 p)
            {
                Vector3 t = p - center;
                return new Vector3(Vector3.Dot(t, orientation[0]), Vector3.Dot(t, orientation[1]),
                                   Vector3.Dot(t, orientation[2]));
            }

            /// <summary>
            /// Convertir un punto de OBB-space a World-space
            /// </summary>
            /// <param name="p">Punto en OBB-space</param>
            /// <returns>Punto convertido a World-space</returns>
            public Vector3 toWorldSpace(Vector3 p)
            {
                return center + p.X*orientation[0] + p.Y*orientation[1] + p.Z*orientation[2];
            }
        }

        /*
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
    }*/
    }

}
