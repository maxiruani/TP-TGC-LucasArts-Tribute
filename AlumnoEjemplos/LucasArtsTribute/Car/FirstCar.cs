using System;
using System.Collections.Generic;
using System.Text;
using TgcViewer.Example;
using TgcViewer;
using Microsoft.DirectX.Direct3D;
using System.Drawing;
using Microsoft.DirectX;
using TgcViewer.Utils.Modifiers;
using TgcViewer.Utils.TgcSceneLoader;
using TgcViewer.Utils.TgcGeometry;
using TgcViewer.Utils.Input;
using Microsoft.DirectX.DirectInput;
using TgcViewer.Utils.TgcSkeletalAnimation;

namespace AlumnoEjemplos.LucasArtsTribute
{
    public class FirstCar
    {
        const float ACELERACION = 1;
        const float VELODICAD_CAMINAR = 250f;
        const float VELOCIDAD_ROTACION = 120f;

        #region Fields

        private float elapsedTime;

        public float ElapsedTime
        {
            get { return elapsedTime; }
            set { elapsedTime = value; }
        }

        private float velocity;

        public float Velocity
        {
            get { return velocity; }
            set { velocity = value; }
        }

        private float acceleration;

        public float Acceleration
        {
            get { return acceleration; }
            set { acceleration = value; }
        }

        public Vector3 Position
        {
            get { return mesh.Position; }
            set { mesh.Position = value; }
        }

        private TgcMesh mesh;

        public TgcMesh Mesh
        {
            get { return mesh; }
            set { mesh = value; }
        }

        // Position

        private float moveForward;

        public float MoveForward
        {
            get { return moveForward; }
            set { moveForward = value; }
        }

        private float rotate;

        private float Rotate
        {
            get { return rotate; }
        }

        // Flags de movimiento

        private bool isMoving;

        public bool IsMoving
        {
            get { return isMoving; }
            set { isMoving = value; }
        }

        private bool isRotating;

        public bool IsRotating
        {
            get { return isRotating; }
            set { isRotating = value; }
        }

        private bool isMovingBack;

        public bool IsMovingBack
        {
            get { return isMovingBack; }
            set { isMovingBack = value; }
        }

        private List<TgcBox> obstacles;

        public List<TgcBox> Obstacles
        {
            get { return obstacles; }
            set { obstacles = value; }
        }

        public Instrumental Instrumental { get; internal set; }
        
        #endregion

        public FirstCar(TgcMesh car_mesh)
        {
            velocity = 0;
            acceleration = 0;
            mesh = car_mesh;
            stats = new CarStats();
            LoadInstrumental();

        }

        private void LoadInstrumental()
        {
            Instrumental = new Instrumental(stats);
        }

        public void NewMove(float time)
        {
            elapsedTime = time;
            moveForward = 0f;
            rotate = 0f;

            isRotating = false;
            isMoving = false;
            isMovingBack = false;
        }

        public void PressedButtons(TgcD3dInput d3dInput)
        {
            // Adelante
            if (d3dInput.keyDown(Key.W))
            {
                isMoving = true;
                moveForward = -VELODICAD_CAMINAR;
            }

            // Atras
            if (d3dInput.keyDown(Key.S))
            {
                isMovingBack = true;
                isMoving = true;
                moveForward = VELODICAD_CAMINAR;
            }

            // Derecha
            if (d3dInput.keyDown(Key.D))
            {
                if (isMovingBack)
                    rotate = -VELOCIDAD_ROTACION;
                else
                    rotate = VELOCIDAD_ROTACION;

                isRotating = true;
            }

            // Izquierda
            if (d3dInput.keyDown(Key.A))
            {
                if (isMovingBack)
                    rotate = VELOCIDAD_ROTACION;
                else
                    rotate = -VELOCIDAD_ROTACION;

                isRotating = true;
            }

        }

        public void Move(TgcD3dInput d3dInput, float elapsedTime)
        {
            // Inicializo variables
            this.NewMove(elapsedTime);

            // Seteo los botones presionados
            this.PressedButtons(d3dInput);

            // Si hubo rotacion
            if (isRotating && isMoving)
            {
                // Rotar personaje y la camara, hay que multiplicarlo por el tiempo transcurrido para no atarse a la velocidad el hardware
                float rotAngle = Geometry.DegreeToRadian(rotate * elapsedTime);

                mesh.rotateY(rotAngle);
                GuiController.Instance.ThirdPersonCamera.rotateY(rotAngle);
            }

            // Si hubo desplazamiento
            if (isMoving)
            {
                // Aplicar movimiento hacia adelante o atras segun la orientacion actual del Mesh
                Vector3 lastPos = mesh.Position;
                mesh.moveOrientedY(moveForward * elapsedTime);
            }
        }


        public Vector3 GetPosition()
        {
            Matrix m = this.mesh.Transform;
            return new Vector3(m.M41, m.M42, m.M43);
        }


        public Vector3 XAxis()
        {
            Matrix m = this.mesh.Transform;
            return new Vector3(m.M11, m.M12, m.M13);
        }

        public Vector3 YAxis()
        {
            Matrix m = this.mesh.Transform;
            return new Vector3(m.M21, m.M22, m.M23);
        }

        public Vector3 ZAxis()
        {
            Matrix m = this.mesh.Transform;
            return new Vector3(m.M31, m.M32, m.M33);
        }

        private CarStats stats;
    }



}
