using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;
using TgcViewer;

//using TgcViewer.Utils;

namespace AlumnoEjemplos.LucasArtsTribute
{
    public class Camara : TgcViewer.Utils.Input.TgcCamera
    {
        public Camara()
        {
            resetValues();
            //GuiController.Instance.CurrentCamera = this;
            camaras = new Queue<Vector3>();
            camaras.Enqueue(nearCamara);
            camaras.Enqueue(farCamara);
            ActualCamara = defaultCamara;
        }


        public void ChangeCamara()
        {
            if ((DateTime.Now - lastCamaraChange).TotalMilliseconds > 500)
            {
                Vector3 camara = new Vector3(ActualCamara.X, ActualCamara.Y, ActualCamara.Z);
                camaras.Enqueue(camara);
                ActualCamara = camaras.Dequeue();
                lastCamaraChange = DateTime.Now;
            }
        }

        private DateTime lastCamaraChange;

        private Queue<Vector3> camaras;

        #region MandatoryMethods
        public Vector3 getPosition()
        {
            return center;
        }

        public Vector3 getLookAt()
        {
            return target;
        }

        public void updateCamera()
        {
            if (!enable)
            {
                return;
            }
            float delta = GuiController.Instance.ElapsedTime * speed;
            if (delta > 1.0f) delta = 1.0f;

            center += (nextCenter - center) * delta;
            target += (nextTarget - target) * delta;
            up += (nextUp - up) * delta;

            viewMatrix = Matrix.LookAtLH(center, target, up);
        }

        public void updateViewMatrix(Device d3dDevice)
        {
            if (!enable)
            {
                return;
            }

            d3dDevice.Transform.View = viewMatrix;
        }
       
        #endregion




        public void SetCenterTargetUp(Vector3 _center, Vector3 _target, Vector3 _up)
        {
            SetCenterTargetUp(_center, _target, _up, false);
        }

        public void SetCenterTargetUp(Vector3 _center, Vector3 _target, Vector3 _up, bool teleport)
        {
            nextCenter = _center;
            nextTarget = _target;
            nextUp = _up;

            if (teleport)
            {
                center = _center;
                target = _target;
                up = _up;

                viewMatrix = Matrix.LookAtLH(center, target, up);
            }
        }


        /// <summary>
        /// Carga los valores default de la camara
        /// </summary>
        private void resetValues()
        {
            up = new Vector3(0.0f, 1.0f, 0.0f);
            center = new Vector3(0, 0, 0);
            target = new Vector3(0, 0, -1.0f);

            nextCenter = center;
            nextTarget = target;
            nextUp = up;

            speed = DEFAULT_SPEED;

            viewMatrix = Matrix.Identity;
        }



        public static float DEFAULT_SPEED = 10.0f;

        bool enable = true;
        public bool Enable { get; set; }

        float speed;

        Vector3 up;
        Vector3 center;
        Vector3 target;

        Vector3 nextCenter;
        Vector3 nextTarget;
        Vector3 nextUp;

        Matrix viewMatrix;

        public Vector3 ActualCamara { get; internal set; }
        private List<Vector3> camaraVectors;
        Vector3 defaultCamara = new Vector3(0, 150, -500);
        Vector3 farCamara = new Vector3(0, 250, -800);
        Vector3 nearCamara = new Vector3(0, 10, -35);

    }
}
