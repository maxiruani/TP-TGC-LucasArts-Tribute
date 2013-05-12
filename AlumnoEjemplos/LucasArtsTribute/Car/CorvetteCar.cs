using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;
using TgcViewer;
using TgcViewer.Utils.TgcGeometry;
using TgcViewer.Utils.TgcSceneLoader;
using System.IO;

namespace AlumnoEjemplos.LucasArtsTribute.Models
{
    public class CorvetteCar
    {
        // ---- Constants ----
        private float rho = 1.29f;              // Density of air (kg/m3)

        // ---- RunTime Variables ----
        private Vector velocity;                // Velocity vector
        private Vector acceleration;            // Acceleration vector
        private Vector position;                // Position vector

        private Vector Ftraction;               // Tracion force vector
        private Vector Fdrag;                   // Drag force vector
        private Vector Frr;                     // Rolling Resistance force vector
        private Vector Fbraking;                // Braking force vector
        private Vector Flong;                   // Long force vector

        // ------ Nuevo -------
        private float wheel_angular_velocity;

        private float angular_velocity;         // Angular Velocity aka Wheel Rotation Rate
        private float angular_acceleration;     // Angular Acceleration
        private float rpm_engine;               // RPM of engine
        private float rpm_wheel;                // RPM of wheels
        private float torque_engine;            // Engine torque
        private float torque_drive;             // Drive torque

        // ---- Sensible to User input ----
        public float throttle;                 // Throttle position
        public float brake;                    // Brake position
        public int gear;                       // Actual gear
        public float angle;                    // Angle
        
        // ---- World coordinates ---- (¿Hacerlos en Fields calculables?)
        private Vector velocity_wc;
        private Vector acceleration_wc;
        private Vector position_wc;
        public Vector Position_wc
        {
            set { position_wc = value; }
            get { return position_wc; }
        }

        // ---- Car Seteable Variables ----
        private float mass = 1500;              // Mass of the car (including driver)
        private float inertia;                  // Rear wheels car inertia
        private float Cb;                       // Braking Constant
        private float Cd = 0.3f;                // Coefficient of friction
        private float A = 2.2f;                 // Frontal Area of the car (m2)
        private float Xd = 3.42f;               // Differential ratio
        private float n = 0.7f;                 // Transmission efficiency
        private float wheel_radius = 0.33f;     // Wheel radius
        private int[,] torque_curve;            // Torque Curve
        private float[] gears_ratio;            // Gears Ratio
        
        // private float[] gears_wheel_rpm_shift;  // Wheel RPM transmission shift
        // ------ Nuevo ------
        private float[] gears_rpm_shift;

        private float[] max_speed_per_gear;     // Max Speed per gear
        private float rpm_red_line;             // Max RPM engine red line

        // ----- Car Mesh
        private TgcMesh mesh;

        public TgcMesh Mesh
        {
            get { return mesh; }
            set { mesh = value; }
        }

        // ---- Calculable Fields On Construction ----
        private float Cdrag;                    // Drag coefficient
        private float Crr;                      // Rolling Resistance coefficient

        // ---- Logger ----
        StreamWriter sw;

        public CorvetteCar()
        {
            // Calculate the Drag coefficient of the car
            Cdrag = 0.5f + Cd + A + rho;

            // Calculate the Rolling Resistance coefficient
            Crr = Cdrag * 30;

            // Calculate the Rear Wheels inertia
            inertia = 1500; // 75 * wheel_radius * wheel_radius;

            // Initialize RunTime variables

            // - Magnitude Vectors
            velocity = new Vector(0, 0, 0);
            acceleration = new Vector(0, 0, 0);
            position = new Vector(0, 0, 0);

            // - World Coordinates Vectors
            velocity_wc = new Vector(0, 0, 0);
            acceleration_wc = new Vector(0, 0, 0);
            Position_wc = new Vector(0, 0, 0);

            // - Forces
            Ftraction = new Vector(0, 0, 0);
            Fdrag = new Vector(0, 0, 0);
            Frr = new Vector(0, 0, 0);
            Fbraking = new Vector(0, 0, 0);
            Flong = new Vector(0, 0, 0);

            // - Magnitude Scalars
            angular_velocity = 0;
            angular_acceleration = 0;

            // - Nuevo
            wheel_angular_velocity = 0;

            // - User variables
            throttle = 0;
            brake = 0;
            gear = 1;                   // First gear
            angle = 0;

            // Initialize the Gear Ratios
            gears_ratio = new float[7];

            //gears_ratio[0] = 2.90f;   // Reverse
            //gears_ratio[0] = 0;       // Neutral
            gears_ratio[1] = 2.66f;     // First gear
            gears_ratio[2] = 1.78f;     // Second gear
            gears_ratio[3] = 1.30f;     // Third gear
            gears_ratio[4] = 1.0f;      // Fourth gear
            gears_ratio[5] = 0.74f;     // Fifth gear
            gears_ratio[6] = 0.5f;      // Sixth gear

            // Define the Wheel RPM when the transmission would shift
            /*
            gears_wheel_rpm_shift = new float[7];
             
            gears_wheel_rpm_shift[1] = 510;     // First gear
            gears_wheel_rpm_shift[2] = 760;     // Second gear
            gears_wheel_rpm_shift[3] = 1100;    // Third gear
            gears_wheel_rpm_shift[4] = 1850;    // Fourth gear
            gears_wheel_rpm_shift[5] = 2800;    // Fifth gear
            gears_wheel_rpm_shift[6] = 3450;    // Six gear top
            */
            gears_rpm_shift = new float[7];

            gears_rpm_shift[1] = 4548;    // First gear
            gears_rpm_shift[2] = 4870;    // Second gear
            gears_rpm_shift[3] = 4890;    // Third gear
            gears_rpm_shift[4] = 5130;    // Fourth gear
            gears_rpm_shift[5] = 4555;    // Fifth gear
            gears_rpm_shift[6] = 5985;    // Six gear top

            // Define the Max Speed per Gear when the transmission would shift
            max_speed_per_gear = new float[7];

            for (int i = 1; i < 7; i++)
                max_speed_per_gear[i] = wheel_radius * 2 * FastMath.PI * gears_rpm_shift[i] / (60 * gears_ratio[i] * Xd);
                //max_speed_per_gear[i] = gears_wheel_rpm_shift[i] * wheel_radius;           
            /*
            max_speed_per_gear[1] = 20;
            max_speed_per_gear[2] = 42;
            max_speed_per_gear[3] = 80;
            max_speed_per_gear[4] = 120;
            max_speed_per_gear[5] = 165;
            max_speed_per_gear[6] = 190;
            */
            // Initialize the Torque Curve
            torque_curve = new int[6, 2];

            torque_curve[0, 0] = 1000;
            torque_curve[0, 1] = 400;
            torque_curve[1, 0] = 2000;
            torque_curve[1, 1] = 440;
            torque_curve[2, 0] = 3000;
            torque_curve[2, 1] = 455;
            torque_curve[3, 0] = 4000;
            torque_curve[3, 1] = 485;
            torque_curve[4, 0] = 5000;
            torque_curve[4, 1] = 478;
            torque_curve[5, 0] = 6000;
            torque_curve[5, 1] = 390;

            // Set the Engine RPM red line
            rpm_red_line = 7200;

            // Reflect values on Screen
            stats = new CarStats();
            acceleration_wc = new Vector(0, 0, 0);
            velocity_wc = new Vector(0, 0, 0);

            LoadInstrumental();

            //Pass the filepath and filename to the StreamWriter Constructor
            sw = new StreamWriter("C:\\TGC-Logs\\log_corvette.txt");
            sw.WriteLine("Instance Car");
        }

        public Instrumental Instrumental { get; internal set; }

        private CarStats stats;

        private void LoadInstrumental()
        {
            Instrumental = new Instrumental(stats);
        }

        public void DoPhysics(float delta_t)
        {
            // Transform Velocity in World Coordinates to Model Coordinates reference
            velocity = velocity_wc.FromAngleToY(angle);

            // Update the Base Values
            UpdateValues();

            // Calculate the Traction Force
            Ftraction = GetTractionForce();

            // Calculate the Drag Force
            Fdrag = GetDragForce();

            // Calculate the Rolling Resistance Force
            Frr = GetRollingResistanceForce();

            // Calculate the Braking Force
            Fbraking = GetBrakingForce();

            // Calculate the Long Force of the car
            Flong = Ftraction + Fdrag + Frr + Fbraking;

            // Apply the Automatic Transsmition
            AutomaticTransmission();

            // Calculate Acceleration of the Model
            acceleration = Flong / this.mass;

            // Calculate the Angular Acceleration
            // angular_acceleration = (torque_drive + (Ftraction.Length() * wheel_radius)) / inertia;

            // Transform Acceleration to World Coordinates
            acceleration_wc = acceleration.FromAngleToY(angle);

            // Calculate the Velocity in World Coordinates
            velocity_wc = velocity_wc + delta_t * acceleration_wc;

            // Calculate the Position in World Coordinates
            Position_wc = Position_wc + delta_t * velocity_wc;

            // Calculate the Angular Velocity
            // angular_velocity = angular_velocity + delta_t * this.angular_acceleration;

            // Update the angle
            // angle = angle + delta_t * angular_velocity;

         //   stats.Acceleration = acceleration_wc.Length();
         //   stats.Velocity = velocity_wc.Length() * 3.6f;
         //   stats.Gear = gear;
            stats.Zmax = this.Mesh.BoundingBox.PMax.Z;
            stats.Zmin = this.Mesh.BoundingBox.PMin.Z;

            sw.WriteLine("-------------------------------");
            sw.WriteLine("Angle: " + this.angle.ToString());
            sw.WriteLine("RPM: " + this.rpm_engine.ToString() + " Lookup Torque: " + LookupTorqueCurve(rpm_engine).ToString());
            sw.WriteLine("Gear: " + this.gear.ToString());
            sw.WriteLine("Ftraction: " + this.Ftraction.Length().ToString());
            sw.WriteLine("Fdrag: " + this.Fdrag.Length().ToString() + " Frr: " + this.Frr.Length().ToString());
            sw.WriteLine("Flong: " + this.Flong.ToString());
            sw.WriteLine("WC Acceleration: " + this.acceleration_wc.Length().ToString());
            sw.WriteLine("WC Velocity: " + this.velocity_wc.Length().ToString());
            
        }

        float tmp = 0;

        public void UpdateValues()
        {
            // 1- Update the Engine RPMs
            //rpm_engine = GetActualEngineRPM();
            rpm_engine = GetEngineTurnOverRate();

            // 2- Update the Wheels RPMs that depends on Engine RPMs
            //rpm_wheel = GetActualWheelRPM();

            // 3- Update the Engine Torque that depends on the Engine RPMs
            torque_engine = GetEngineTorque();

            // 4- Update the Drive Torque that depends on the Engine Torque
            torque_drive = GetDriveTorque();
        }

        // Drag Force
        public Vector GetDragForce()
        {
            return - Cdrag * velocity * velocity.Length();
        }

        // Rolling Resistance Force
        public Vector GetRollingResistanceForce()
        {
            return - Crr * velocity;
        }

        // Braking Force
        public Vector GetBrakingForce()
        {
            return - new Vector(Cb, 0, 0);
        }

        // Engine Force
        public Vector GetTractionForce()
        {
            // return new Vector(throttle * torque_drive / wheel_radius, 0, 0);
            return - new Vector((throttle * torque_drive) / wheel_radius, 0, 0);
        }

        public float GetDriveTorque()
        {
            //engine_torque = GetEngineTorque();
            return torque_engine * gears_ratio[gear] * Xd *n;
        }

        public float GetEngineTorque()
        {
            //rpm = GetActualEngineRPM();
            //return throttle * LookupTorqueCurve(rpm_engine);
            return LookupTorqueCurve(rpm_engine);
        }
        
        
        public float LookupTorqueCurve(float rpm)
        {
            int len = torque_curve.Length/2;
            for (int i = 0; i < len; i++)
            {
                if (rpm <= torque_curve[i, 0])
                {
                    // Si es el primer punto de la curva, devuelvo la base de RPMs
                    if (i == 0)
                        return torque_curve[0, 1];

                    // Si no, promedio entre los dos puntos
                    return (torque_curve[i - 1, 1] + torque_curve[i, 1]) / 2;
                }

                // Si es el ultimo, devuelvo el tope de corte
                if (i == (len - 1))
                    return torque_curve[len - 1, 1];
            }

            return 0;
        }
        
        /*
        public float LookupTorqueCurve(float rpm)
        {
            int len = torque_curve.Length / 2;
            for (int i = 0; i < len; i++)
            {
                if (rpm <= torque_curve[i, 0])
                {
                    // Si es el primer punto de la curva, devuelvo la base de RPMs
                    if (i == 0)
                        return torque_curve[0, 1];

                    // Si no, promedio entre los dos puntos
                    return torque_curve[i, 1];
                }

                // Si es el ultimo, devuelvo el tope de corte
                if (i == (len - 1))
                    return torque_curve[len - 1, 1];
            }

            return 0;
        }
        */
        /* ------------ Viejo -------------
        public float GetActualEngineRPM()
        {
            float temp = angular_velocity * gears_ratio[gear] * Xd * 60 / 2 * FastMath.PI;
            
            if (temp > rpm_red_line)
                return rpm_red_line;

            if (temp < 1000)
                return 1000;

            return temp;
        }

        public float GetActualWheelRPM()
        {
            //return GetActualEngineRPM() / (gear * Xd);
            //return rpm_engine / (gear * Xd);
            return velocity.X * 60 * gears_ratio[gear] * Xd / FastMath.PI * wheel_radius;
        }
        
        public float GetAngularVelocity()
        {
            return (velocity.Length() * 1000 / 3600) / wheel_radius;
        }
        */

        // ------------- Nuevo ----------------
        // Engine RPM
        public float GetEngineTurnOverRate()
        {
            float temp = FastMath.Abs((velocity.X * 60 * gears_ratio[gear] * Xd) / (2 * FastMath.PI * wheel_radius));
            tmp = temp;
            
            //float a_velocity = (velocity.Length() * 1000) / 3600;
            //float temp = a_velocity * 60 * gears_ratio[gear] * Xd / 2 * FastMath.PI;

            if (temp > rpm_red_line)
                return rpm_red_line;

            if (temp < 1000)
                return 1000;

            return temp;
        }

        // Velocidad angular de la Rueda
        public float GetWheelAngularVelocity()
        {
            return 2 * FastMath.PI * GetEngineTurnOverRate() / 60 * gears_ratio[gear] * Xd;
        }

        // ------------------------------------

        public void AutomaticTransmission()
        {
            if (velocity.Length() > max_speed_per_gear[gear])
            {
                gear++;
                return;
            }

            if (gear == 1)
                return;

            if (velocity.Length() < max_speed_per_gear[gear - 1])
            {
                gear--;
                return;
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

        public void CrashWithObject()
        {
            velocity_wc = new Vector(0,0,0);
        }

        public void Render()
        {
            // Set the new position
            mesh.Position = this.Position_wc.ToDirectXVector();

            // Set the rotation
            //mesh.Rotation = new Vector3(0, FastMath.PI + this.angle, 0);

            // Render the car
            mesh.render();
        }

        public void RenderEffect()
        {
            Effect effect = mesh.Effect;
            effect.SetValue("fvLightPosition", TgcParserUtils.vector3ToFloat3Array(new Vector3(0, 500, 0)));
            effect.SetValue("fvEyePosition",
                            TgcParserUtils.vector3ToFloat3Array(GuiController.Instance.ThirdPersonCamera.getPosition()));
            effect.SetValue("k_la", (float) GuiController.Instance.Modifiers["Ambient"]);
            effect.SetValue("k_ld", (float) GuiController.Instance.Modifiers["Diffuse"]);
            effect.SetValue("k_ls", (float) GuiController.Instance.Modifiers["Specular"]);
            effect.SetValue("fSpecularPower", (float) GuiController.Instance.Modifiers["SpecularPower"]);
        }
    }
}
