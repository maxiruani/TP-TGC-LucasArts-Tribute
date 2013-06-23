using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using AlumnoEjemplos.LucasArtsTribute.VehicleModel;
using Microsoft.DirectX;
using TgcViewer;
using TgcViewer.Utils.TgcGeometry;
using TgcViewer.Utils.TgcSceneLoader;

namespace AlumnoEjemplos.LucasArtsTribute
{
    class OrientedBoundingBox
    {
        
        public  bool isBoundingBox()
        {
            return true;
        }
        // position_wc_o = v_position
        // velocity_wc_o = v_velocity
        // angle_o = car.Body.Rotation.Y
        // angularVelocity = car.s_omega
        // size = car.Body.Scale
        private Vector3 _positionWc;
        private Vector3 _velocityWc;
        private float _angle;
        private float _angularVelocity;
        private Vector3 _size;
        private Vehicle _car;

        public OrientedBoundingBox(Vehicle car)
        {
            _car = car;
            _positionWc = car.VPosition;
            _velocityWc = car.VVelocity;
            _angle = car.body.Rotation.Y;
            _angularVelocity = car.SOmega;
            _size = car.body.Scale;
        }

        public OrientedBoundingBox Move()
        {
            _positionWc = _car.VPosition;
            _velocityWc = _car.VVelocity;
            _angle = _car.body.Rotation.Y;
            _angularVelocity = _car.SOmega;
            _size = _car.body.Scale;
            return this;
        }
        /*
        private delegate Contact Distance(float t);

        private Contact iterativeSwapTest(Distance getDistance, Accelerationable obstaculo, Contact contact, float delta_t)
        {
            if (contact.tMax > -0.1f) // LAS ESFERAS CHOCAN
            {
                // ciclos
                float i = 0;

                // tiempos de colision
                float t = contact.t;
                float tMin = contact.t;
                float tMax = contact.tMax;

                // velocidad relativa
                Vector3 ray = obstaculo.velocity_wc - this.velocity_wc;

                // contacto inicial
                contact = getDistance(t);

                while (Math.Abs(contact.dist) > 0.01f && i++ < 100)
                {
                    // castear velocidad contra normal
                    float rayN = ray * contact.normal;
                    float dRel = rayN + this.radious * Math.Abs(this.angular_velocity);

                    // avance conservativo del tiempo
                    t += contact.dist / dRel;

                    if (t < tMin || t > tMax)
                    {
                        // nunca colisionan...
                        contact.t = -1;
                        return contact;
                    }

                    // iterar
                    contact = getDistance(t);
                }

                contact.t = t;

                if ((bool)GuiController.Instance.Modifiers["Predicción de choque"])
                {
                    this.Move(t).render(System.Drawing.Color.LightBlue);
                }

                //obstaculo.move(t).render(System.Drawing.Color.LightGreen);
            }
            else
            {
                contact.t = -1;
            }
            
            return contact;
        }

        public  Contact swapTest(Accelerationable obstaculo, float delta_t)
        {
            return obstaculo.swapTest(this, delta_t);
        }

        public  Contact swapTest(BoundingSphere obstaculo, float delta_t)
        {
            return this.iterativeSwapTest(delegate(float t) { return this.Move(t).getDistance(obstaculo.move(t)); }, obstaculo, base.swapTest(obstaculo, delta_t), delta_t);
        }

        public  Contact swapTest(OrientedBoundingBox obstaculo, float delta_t)
        {
            return this.iterativeSwapTest(delegate(float t) { return this.Move(t).getDistance(obstaculo.Move(t)); }, obstaculo, base.swapTest((BoundingSphere)obstaculo, delta_t), delta_t);
        }

        public  Contact swapTest(InfiniteWall obstaculo, float delta_t)
        {
            return this.iterativeSwapTest(delegate(float t) { return this.Move(t).getDistance(obstaculo); }, obstaculo, base.swapTest(obstaculo, delta_t), delta_t);
        }*/

        // RENDERIZABLE

        public  void Render()
        {
            //TgcTexture textura = TgcTexture.createTexture(GuiController.Instance.D3dDevice, GuiController.Instance.AlumnoEjemplosMediaDir + "SeniorCoders\\paredMuyRugosa.jpg");
            TgcBox caja = TgcBox.fromExtremes((-this._size), this._size);//, textura);
            caja.Position = this._positionWc;
            caja.Rotation = new Vector3(0, FastMath.PI + _angle, 0);
            caja.render();
        }

        public  void Render(System.Drawing.Color color)
        {
            //base.render(color);

            TgcBox caja = TgcBox.fromSize(new Vector3(0,0,0), new Vector3((float) 0.1, (float) 0.1, (float) 0.1), color); //TgcBox.fromExtremes((-this._size), this._size, color);
            //caja.Position = this._positionWc;
            //caja.Rotation = new Vector3(0, FastMath.PI + _angle, 0);
            caja.Enabled = true;
            caja.render();
        }
    
    }
    
}
