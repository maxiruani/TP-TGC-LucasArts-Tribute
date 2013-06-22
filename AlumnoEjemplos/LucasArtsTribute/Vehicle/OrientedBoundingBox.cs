using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.DirectX;
using TgcViewer;
using TgcViewer.Utils.TgcGeometry;
using TgcViewer.Utils.TgcSceneLoader;

namespace AlumnoEjemplos.LucasArtsTribute
{
    class OrientedBoundingBox
    {
        /*
        public Vector3 size; // Tamaño de la caja
        
        public  bool isBoundingBox()
        {
            return true;
        }
        // position_wc_o = v_position
        // velocity_wc_o = v_velocity
        // angle_o = car.Body.Rotation.Y
        // angular_velocity_o = car.s_omega
        // size = car.Body.Scale
        public OrientedBoundingBox(Vector3 position_wc_o, Vector3 velocity_wc_o, float angle_o, float angular_velocity_o, Vector3 size)
            : base(position_wc_o, velocity_wc_o, size.length())
        {
            this.size = size;
            _car.
            this.angle = angle_o;
            this.angular_velocity = angular_velocity_o;
        }

        public OrientedBoundingBox move(float delta_t)
        {
            OrientedBoundingBox OBB = new OrientedBoundingBox(this.position_wc, this.velocity_wc, this.angle, this.angular_velocity, this.size);
            
            OBB.position_wc = this.position_wc + this.velocity_wc * delta_t;
            OBB.angle = this.angle + this.angular_velocity * delta_t;
            
            return OBB;
        }

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
                    this.move(t).render(System.Drawing.Color.LightBlue);
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
            return this.iterativeSwapTest(delegate(float t) { return this.move(t).getDistance(obstaculo.move(t)); }, obstaculo, base.swapTest(obstaculo, delta_t), delta_t);
        }

        public  Contact swapTest(OrientedBoundingBox obstaculo, float delta_t)
        {
            return this.iterativeSwapTest(delegate(float t) { return this.move(t).getDistance(obstaculo.move(t)); }, obstaculo, base.swapTest((BoundingSphere)obstaculo, delta_t), delta_t);
        }

        public  Contact swapTest(InfiniteWall obstaculo, float delta_t)
        {
            return this.iterativeSwapTest(delegate(float t) { return this.move(t).getDistance(obstaculo); }, obstaculo, base.swapTest(obstaculo, delta_t), delta_t);
        }

        // RENDERIZABLE

        public  void render()
        {
            //base.render();

            TgcTexture textura = TgcTexture.createTexture(GuiController.Instance.D3dDevice, GuiController.Instance.AlumnoEjemplosMediaDir + "SeniorCoders\\paredMuyRugosa.jpg");
            TgcBox caja = TgcBox.fromExtremes((-this.size).toDirectXVector3(), this.size.toDirectXVector3(), textura);
            caja.Position = this.position_wc.toDirectXVector3();
            caja.Rotation = new Vector33(0, FastMath.PI + this.angle, 0);
            caja.render();
        }

        public  void render(System.Drawing.Color color)
        {
            //base.render(color);

            TgcBox caja = TgcBox.fromExtremes((-this.size).toDirectXVector3(), this.size.toDirectXVector3(), color);
            caja.Position = this.position_wc.toDirectXVector3();
            caja.Rotation = new Vector33(0, FastMath.PI + this.angle, 0);
            caja.render();
        }
    */
    }
    
}
