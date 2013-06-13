using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.DirectX;
using TgcViewer;
using TgcViewer.Utils.TgcGeometry;
using TgcViewer.Utils.TgcSceneLoader;

namespace AlumnoEjemplos.LucasArtsTribute.Car
{
    public class Model
    {
        // Constants
        private float GRAVITY = 9.8f;

        // Car Mesh
        private TgcMesh mesh;

        public TgcMesh Mesh
        {
            get { return mesh; }
            set { mesh = value; }
        }

        // - Configurable

        private float wheelbase;        // b + c in m 
        private float b;                // In m, distance from CG to front axle. Distancia desde el Centro de gravedad al eje frontal.
        private float c;                // In m, distance from CG to rear axle. Distancia desde el Centro de gravedad al eje trasero.
        private float h;                // In m, height of CM from ground. Distancia desde el Centro de gravedad al piso.
        private float mass;             // In kg
        private float inertia;          // In kg.m
        private float wheellength;
        private float wheelwidth;
        private float weight;

        // - Configurable Constants

        private float DRAG;             // Factor for air resistance (drag). Factor de resistencia al aire.
        private float RESISTANCE;       // Factor for rolling resistance.
        private float CA_R = -5.20f;    // Cornering stiffness rear.
        private float CA_F = -5.0f;     // Cornering stiffness front.
        private float MAX_GRIP = 2.0f;  // Maximum (normalised) friction force.

        public float STEER_ANGLE_MAX = 0.015f;      // Tope de giro del volante
        public float STEER_ANGLE_RATIO = 0.0001f;   // Ratio de giro (Cuanto mas grande, mas rapido llega al Tope y mas rapido dobla.
        public float STEER_RATIO_BACK =  0.00001f;
        // Input
        public float steerangle;        // Angle of steering. Angulo de giro del volante.
        public float throttle;          // Amount of throttle. Pedal de acelerador.
        public float brake;             // Amount of brake. Pedal de freno.

        // RunTime
        private float angle;            // Angle of car body orientation (in rads). Angulo de orientacion del cuerpo.
        private float car_torque;       // Torque del auto debido a las fuerzas laterales

        private float angular_acceleration;     // Aceleracion angular del auto (del modelo)
        private float angular_velocity;         // Velocidad angular del auto (del modelo)
        private float yaw_speed;                // Velocidad del chasis, resultante de la velocidad angular del modelo

        private Vector acceleration_wc;        // Vector de aceleracion en coordenadas del mundo
        private Vector acceleration;           // Vector de aceleracion en coordenadas del modelo

        private Vector velocity_wc;            // Vector de velocidad en coordenadas del mundo.
        private Vector velocity;               // Vector de velocidad en coordenadas del modelo.

        private Vector position_wc;            // Vector de posicion en coordenadas del mundo.

        private float rotation_angle;           // Angulo de rotacion
        private float side_slip_angle;          // Side slip angle. Beta
        private float slip_angle_front;         // Alpha front
        private float slip_angle_rear;          // Alpha rear

        public int front_slip;
        public int rear_slip;

        // Forces
        private Vector force;                  // Fuerza total
        private Vector f_resistance;           // Fuerza de resistencia
        private Vector f_traction;             // Fuerza de traccion
        private Vector f_lat_front;            // Fuerza lateral frontal
        private Vector f_lat_rear;             // Fuerza lateral trasera

        public Model()
        {
            this.b = 1;
            this.c = 1;
            this.wheelbase = this.b + this.c;
            this.h = 1;
            this.mass = 1500;
            this.inertia = 1500;
            this.wheellength = 0.7f;
            this.wheelwidth = 0.3f;

            this.DRAG = 1;
            this.RESISTANCE = 10;
            //this.DRAG = 5;
            //this.RESISTANCE = 30;  
            this.CA_R = -5.20f; 
            this.CA_F = -5.0f;    
            this.MAX_GRIP = 2.0f;

            InitializeVariables();
        }

        private void InitializeVariables()
        {
            // Vectors
            position_wc = new Vector(0, 0, 0);

            acceleration_wc = new Vector(0, 0, 0);
            acceleration = new Vector(0, 0, 0);

            velocity_wc = new Vector(0, 0, 0);
            velocity = new Vector(0, 0, 0);

            // Floats
            angle = 0; //FastMath.PI / 4.0f;
            angular_acceleration = 0;
            angular_velocity = 0;

            weight = mass * GRAVITY * 0.5f;

            // Input
            steerangle = 0;
            throttle = 0;
            brake = 0;

            // Forces
            force = new Vector(0,0,0);
            f_resistance = new Vector(0, 0, 0);
            f_traction = new Vector(0, 0, 0);
            f_lat_front = new Vector(0, 0, 0);
            f_lat_rear = new Vector(0, 0, 0);
        }

        public void DoPhysics(float delta_t)
        {
            // Velocidad resultante en las ruedas por la velocidad angular (yaw) del chasis
            // v = velocidad angular * distancia de la rueda al CG (aprox la mitad del wheel base)
            yaw_speed = wheelbase * 0.5f * angular_velocity;

            if (FastMath.Abs(velocity.X) < 1)
            {
                slip_angle_front = 0;
                slip_angle_rear = 0;
            }
            else
            {
                rotation_angle = FastMath.Atan2(yaw_speed, FastMath.Abs(velocity.X));

                // Calculamos beta
                side_slip_angle = FastMath.Atan2(velocity.Z, FastMath.Abs(velocity.X));

                // Calculamos alpha para cada eje (en realidad funciona como una bicicleta gorda y no como un cuatriciclo =P)
                slip_angle_front = side_slip_angle + rotation_angle - steerangle * Math.Sign(velocity.X);
                slip_angle_rear = side_slip_angle - rotation_angle;
            }

            // Peso por eje = la mitad del peso total = masa * gravedad <-- no hay distribucion de peso por aceleracion 
            weight = mass * 9.8f * 0.5f;

            // Fuerza lateral en el eje delantero = Ca * beta, encerrado en el circulo de friccion por peso
            f_lat_front.X = 0;
            f_lat_front.Z = CA_F * slip_angle_front;
            f_lat_front.Z = FastMath.Min( MAX_GRIP, f_lat_front.Z);
            f_lat_front.Z = FastMath.Max(-MAX_GRIP, f_lat_front.Z);
            f_lat_front.Z *= weight;
            f_lat_front.Z *= (float) Math.Cos(steerangle);
            
            // Fuerza lateral en el eje trasero, idem
            f_lat_rear.X = 0;
            f_lat_rear.Z = CA_R * slip_angle_rear;
            f_lat_rear.Z = FastMath.Min( MAX_GRIP, f_lat_rear.Z);
            f_lat_rear.Z = FastMath.Max(-MAX_GRIP, f_lat_rear.Z);
            f_lat_rear.Z *= weight;
            
            // Fuerza longitudinal en el eje trasero (no hay traccion en las 4 ruedas)
            f_traction.X = 100 * throttle * 2;//((this.throttle * 50) - this.brake * Math.Sign(velocity.X));
            f_traction.Z = 0;
            
            // Resistencia por viento y friccion
            f_resistance = -(RESISTANCE * velocity + DRAG * velocity * velocity.Length());

            // La fuerza total es la suma de la traccion + las fuerzas laterales + la resistencia
            force = f_traction + f_lat_rear + f_lat_front + f_resistance;
           
            // Torque en el chasis debido a las fuerzas laterales
            car_torque = b * f_lat_front.Z - c * f_lat_rear.Z;

            // Aceleracion = F / m
            acceleration = force / mass; // Los vectores no admiten / ><
            
            // Aceleracion angular = torque / momento angular
            angular_acceleration = car_torque / inertia;

            // transformamos aceleracion desde las coordenadas del cuerpo a coordenadas del mundo
            acceleration_wc = acceleration.FromAngleToY(angle);

            // velocidad = integral de la acelacion - usamos el metodo de euler para integrar
            velocity_wc += delta_t * acceleration_wc;

            // velocidad angular = integral de la aceleracion angular
            angular_velocity += delta_t * angular_acceleration;

            // actualizamos la velocidad en coordenadas del cuerpo
            velocity = velocity_wc.FromAngleToY(angle);

            // posicion = integral de la velocidad - idem
            position_wc += delta_t * velocity_wc;

            // angulo = integral de la velocidad angular
            angle += delta_t * angular_velocity;
        }

        /*
        public void DoPhysics(float elapsed_time)
        {
            // Transformo el vector velocidad de las coordenadas del mundo
            // a las coordenadas en referencia al modelo del auto.

            float sn = FastMath.Sin(angle);
            float cs = FastMath.Cos(angle);

            velocity.X =  cs * velocity_wc.Z + sn * velocity_wc.X;
            velocity.Z = -sn * velocity_wc.Z + cs * velocity_wc.X;

            // Calculo la velocidad resultante del modelo en base a la velocidad angular del chasis.
            // TODO: Cambiar el 0.5f para el Alpha Front y el Alpha Rear
            yaw_speed = wheelbase * 0.5f * angular_velocity;

            if (FastMath.Abs(velocity.X) < 1)
            {
                rotation_angle = 0;
                side_slip_angle = 0;
            }
            else
            {
                // TODO: Rotation angle Front - Rotation Angle Rear
                rotation_angle = FastMath.Atan2(yaw_speed, FastMath.Abs(velocity.X));
                // Calculo Beta
                side_slip_angle = FastMath.Atan2(velocity.Z, FastMath.Abs(velocity.X));
            }
            
            // Calculo los Alpha para el eje delantero y trasero (front and rear)
            slip_angle_front = side_slip_angle + rotation_angle - steerangle * Math.Sign(velocity.X); ;
            slip_angle_rear  = side_slip_angle - rotation_angle;
            

            // Fuerza lateral en las ruedas delateras. (front)
            f_lat_front.X = 0;
            f_lat_front.Z = CA_F * slip_angle_front;
            f_lat_front.Z = FastMath.Min(MAX_GRIP, f_lat_front.Z);
            f_lat_front.Z = FastMath.Max(-MAX_GRIP, f_lat_front.Z);
            f_lat_front.Z *= weight;
            if (front_slip == 1)
                f_lat_front.Z *= 0.5f;

            // Fuerza lateral en las ruedas traseras. (rear)
            f_lat_rear.X = 0;
            f_lat_rear.Z = CA_R * slip_angle_rear;
            f_lat_rear.Z = FastMath.Min(MAX_GRIP, f_lat_rear.Z);
            f_lat_rear.Z = FastMath.Max(-MAX_GRIP, f_lat_rear.Z);
            f_lat_rear.Z *= weight;
            if (rear_slip == 1)
                f_lat_rear.Z *= 0.5f;

            // Fuerza longitudinal en las ruedas traseras
            f_traction.X = 100 * (throttle - brake * Math.Sign(velocity.X));
            f_traction.Z = 0;

            if (rear_slip == 1)
                f_traction.X *= 0.5f;

            // Fuerzas de resistencia (Drag and Rolling Resistance)
            f_resistance.X = -(RESISTANCE * velocity.X + DRAG * velocity.X * FastMath.Abs(velocity.X));
            f_resistance.Z = -(RESISTANCE * velocity.Z + DRAG * velocity.Z * FastMath.Abs(velocity.Z));

            // Fuerza total
            force.X = f_traction.X + FastMath.Sin(steerangle) * f_lat_front.X + f_lat_rear.X + f_resistance.X;
            force.Z = f_traction.Z + FastMath.Cos(steerangle) * f_lat_front.Z + f_lat_rear.Z + f_resistance.Z;

            // Vector de aceleracion del auto.
            acceleration.X = force.X / mass;
            acceleration.Z = force.Z / mass;

            // Transformo la aceleracion en coordenadas del modelo, a coordenadas del mundo
            acceleration_wc.X =  cs * acceleration.Z + sn * acceleration.X;
            acceleration_wc.Z = -sn * acceleration.Z + cs * acceleration.X;

            // Calculo la velocidad respecto de la aceleracion
            velocity_wc.X += elapsed_time * acceleration_wc.X;
            velocity_wc.Z += elapsed_time * acceleration_wc.Z;

            // Calculo la nueva posicion respecto de la velocidad
            position_wc.X += elapsed_time * velocity_wc.X;
            position_wc.Z += elapsed_time * velocity_wc.Z;

            // Torque del auto respecto a las fuerzas laterales
            car_torque = b * f_lat_front.Z - c * f_lat_rear.Z;

            // Aceleracion angular del auto (modelo).
            angular_acceleration = car_torque / inertia;

            // Calculo la velocidad angular del auto (modelo) respecto de la aceleracion angular
            angular_velocity += elapsed_time * angular_acceleration;

            // Calculo el nuevo angulo del auto (modelo) respecto de la velocidad angular
            angle += elapsed_time * angular_velocity;
        }
        */

        public void Render()
        {
            // Set the new position
            mesh.Position = this.position_wc.ToDirectXVector();

            // Set the rotation
            mesh.Rotation = new Vector3(0, FastMath.PI + this.angle, 0);

            // Render the car
            mesh.render();
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

    }
}
