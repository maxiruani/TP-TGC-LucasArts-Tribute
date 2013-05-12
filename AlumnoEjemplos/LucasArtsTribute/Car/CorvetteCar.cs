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
        private float gravity = 9.8f;           // Gravity
        //private float load = 5000;

        // ---- RunTime Variables ----

        // ---- Vectors
        private Vector velocity;                // Velocity vector
        private Vector acceleration;            // Acceleration vector
        private Vector position;                // Position vector

        // Forces
        private Vector Ftraction;               // Tracion force vector
        private Vector Fdrag;                   // Drag force vector
        private Vector Frr;                     // Rolling Resistance force vector
        private Vector Fbraking;                // Braking force vector
        private Vector Flong;                   // Long force vector

        private Vector Flat_rear;               // Lateral rear wheel force vector
        private Vector Flat_front;              // Lateral front wheel force vector

        // ---- World coordinates ---- (¿Hacerlos en Fields calculables?)
        private Vector velocity_wc;
        private Vector acceleration_wc;
        private Vector position_wc;

        public Vector Position_wc
        {
            get { return position_wc; }
            set { position_wc = value; }
        }

        // ------ Magnitudes -------
        private float wheel_angular_velocity;   // Wheel angular velocity
        private float angular_velocity;         // Car angular velocity
        private float angular_acceleration;     // Car angular acceleration
        private float rpm_engine;               // RPM of engine
        private float rpm_wheel;                // RPM of wheels
        private float torque_engine;            // Engine torque
        private float torque_wheel;             // Drive torque
        private float torque_car;               // Torque which causes the car body to turn

        // ---- Cornering
        private float side_slip_angle;          // Car side slip angle
        private float wheel_front_slip_angle;   // Front wheel slip angle
        private float wheel_rear_slip_angle;    // Rear wheel slip angle

        // ---- Sensible to User input ----
        public float throttle;                  // Throttle position
        public float brake;                     // Brake position
        public int gear;                        // Actual gear
        public float steering_angle;            // Steering angle of rear wheels
        public float yaw_angle;                 // Yaw angle

        // ---- Car Seteable Variables Config ----
        private float mass = 1500;              // Mass of the car (including driver)
        private float Cb = 5000;                       // Braking constant
        private float Cd = 0.3f;                // Coefficient of friction
        private float A = 2.2f;                 // Frontal Area of the car (m2)
        private float Xd = 3.42f;               // Differential ratio
        private float n = 0.7f;                 // Transmission efficiency
        private float wheel_radius = 0.33f;     // Wheel radius
        private int[,] torque_curve;            // Torque Curve
        private float[] gears_ratio;            // Gears Ratio
        private float[] gears_rpm_shift;        // RPM where the gear would shift
        private float[] max_speed_per_gear;     // Max Speed per gear
        private float rpm_red_line = 7200;      // Max RPM engine red line

        private float Ca_front = -5.2f;         // Cornering stiffness constant for front wheels
        private float Ca_rear = -5;             // Cornering stiffness constant for rear wheels
        private float b = 1.265633f;            // Distance from CG to the front axle
        private float c = 1.519292f;            // Distance from CG to the rear axle
        private float max_friction = 3;         // Maximun normalized friction force
        private float inertia = 1500;           // Car inertia

        // ----- Car Mesh
        private TgcMesh mesh;

        public TgcMesh Mesh
        {
            get { return mesh; }
            set { mesh = value; }
        }

        // ---- Calculable Fields On Construction ----
        private float Cdrag = 0.4257f;          // Drag coefficient
        private float Crr = 12.8f;              // Rolling Resistance coefficient
        private float weight;                   // Weight of car

        // ---- Logger ----
        StreamWriter sw;

        public CorvetteCar()
        {
            // Calculate the Drag coefficient of the car
            // Cdrag = 0.5f + Cd + A + rho;

            // Calculate the Rolling Resistance coefficient
            // Crr = Cdrag * 30;

            // Calculate the weight of the car
            weight = mass * gravity;

            // Initialize RunTime variables

            // - Magnitude Vectors
            velocity = new Vector(0, 0, 0);
            acceleration = new Vector(0, 0, 0);
            position = new Vector(0, 0, 0);

            // - World Coordinates Vectors
            velocity_wc = new Vector(0, 0, 0);
            acceleration_wc = new Vector(0, 0, 0);
            position_wc = new Vector(0, 0, 0);

            // - Forces
            Ftraction = new Vector(0, 0, 0);
            Fdrag = new Vector(0, 0, 0);
            Frr = new Vector(0, 0, 0);
            Fbraking = new Vector(0, 0, 0);
            Flong = new Vector(0, 0, 0);
            Flat_front = new Vector(0, 0, 0);
            Flat_rear = new Vector(0, 0, 0);

            // - Magnitudes
            angular_velocity = 0;
            angular_acceleration = 0;
            wheel_angular_velocity = 0;

            side_slip_angle = 0;
            wheel_front_slip_angle = 0;
            wheel_rear_slip_angle = 0;

            // - User variables
            throttle = 0;
            brake = 0;
            gear = 1;
            yaw_angle = 0;

            // Initialize the Gear Ratios
            gears_ratio = new float[7];

            //gears_ratio[0] = 2.90f;   // Reverse
            //gears_ratio[0] = 0;       // Neutral
            gears_ratio[1] = 3.82f;     // First gear
            gears_ratio[2] = 2.20f;     // Second gear
            gears_ratio[3] = 1.52f;     // Third gear
            gears_ratio[4] = 1.22f;     // Fourth gear
            gears_ratio[5] = 1.02f;     // Fifth gear
            gears_ratio[6] = 0.84f;     // Sixth gear

            // Define the Engine RPM when the transmission would shift
            gears_rpm_shift = new float[7];

            gears_rpm_shift[1] = 4000;    // First gear
            gears_rpm_shift[2] = 4000;    // Second gear
            gears_rpm_shift[3] = 4000;    // Third gear
            gears_rpm_shift[4] = 4000;    // Fourth gear
            gears_rpm_shift[5] = 4000;    // Fifth gear
            gears_rpm_shift[6] = 4000;    // Six gear top

            /*
            gears_rpm_shift[1] = 4548;    // First gear
            gears_rpm_shift[2] = 4870;    // Second gear
            gears_rpm_shift[3] = 4890;    // Third gear
            gears_rpm_shift[4] = 5130;    // Fourth gear
            gears_rpm_shift[5] = 4555;    // Fifth gear
            gears_rpm_shift[6] = 5985;    // Six gear top
            */

            // Define the Max Speed per Gear when the transmission would shift
            max_speed_per_gear = new float[7];

            for (int i = 1; i < 7; i++)
                max_speed_per_gear[i] = wheel_radius * 2 * FastMath.PI * gears_rpm_shift[i] / (60 * gears_ratio[i] * Xd);

            // Initialize the Torque Curve
            torque_curve = new int[12, 2];

            torque_curve[0, 0] = 1000;
            torque_curve[0, 1] = 200;
            torque_curve[1, 0] = 1500;
            torque_curve[1, 1] = 230;
            torque_curve[2, 0] = 2000;
            torque_curve[2, 1] = 245;
            torque_curve[3, 0] = 2500;
            torque_curve[3, 1] = 260;
            torque_curve[4, 0] = 3000;
            torque_curve[4, 1] = 270;
            torque_curve[5, 0] = 3500;
            torque_curve[5, 1] = 280;
            torque_curve[6, 0] = 4000;
            torque_curve[6, 1] = 290;
            torque_curve[7, 0] = 4500;
            torque_curve[7, 1] = 300;
            torque_curve[8, 0] = 5000;
            torque_curve[8, 1] = 300;
            torque_curve[9, 0] = 5500;
            torque_curve[9, 1] = 290;
            torque_curve[10, 0] = 6000;
            torque_curve[10, 1] = 265;
            torque_curve[11, 0] = 6500;
            torque_curve[11, 1] = 235;

            /*
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
            */

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
            // Transform velocity in world coordinates to model coordinates reference
            velocity = velocity_wc.FromAngleToY(yaw_angle);

            float yaw_speed = 0.5f * 2.785f * angular_velocity;

            if (FastMath.Abs(velocity.Length()) < 1)
            {
                velocity.X = 0;
                velocity.Y = 0;
                velocity.Z = 0;
            }

            // Compute the slip angles for the front and rear wheels and the car slip angle
            side_slip_angle = GetCarSideSlipAngle();
            wheel_front_slip_angle = /*side_slip_angle + */GetFrontWheelSlipAngle();
            wheel_rear_slip_angle = /*side_slip_angle -*/ GetRearWheelSlipAngle();

            // Compute the lateral forces for front and rear wheels
            Flat_front.Z = GetFrontLateralForce();
            Flat_rear.Z = GetRearLateralForce();

            // Compute the engine turn over rate (Engine RPM)
            rpm_engine = GetEngineTurnOverRate();

            // Automatic transmission
            AutomaticTransmission();

            // Compute engine torque that depends on the Engine RPMs
            torque_engine = GetEngineTorque();

            // Compute the drive torque that depends on the Engine Torque
            torque_wheel = GetWheelTorque();

            // Calculate the Traction Force
            Ftraction = GetTractionForce();

            // If the car is braking
            if (brake > 0)
            {
                //if (gear > 0)
                Ftraction = GetBrakingForce();

                //if (gear == -1)
                //    Fbraking = -GetBrakingForce();
            }

            // If the car is reversing
            if (gear == -1)
            {

            }

            // Calculate the rolling resistance rorce
            Frr = GetRollingResistanceForce();

            // Calculate the drag force
            Fdrag = GetDragForce();

            // Calculate the Long Force of the car
            Flong = Ftraction /*+ Fbraking*/ + Frr + Fdrag + Flat_front + Flat_rear;

            //Flong.X = Ftraction.X + Flat_front * FastMath.Sin(steering_angle) * (Fdrag.X + Frr.X);
            //Flong.Z =   Flat_rear + Flat_front * FastMath.Cos(steering_angle) * (Fdrag.Z + Frr.Z);

            // Calculate the torque on the car body
            torque_car = FastMath.Cos(steering_angle) * Flat_front.Z * b - Flat_rear.Z * c;

            // Calculate acceleration of the model
            acceleration = Flong / this.mass;

            // Calculate the angular acceleration of the car
            angular_acceleration = torque_car / inertia;

            // Transform acceleration to world coordinates
            acceleration_wc = acceleration.FromAngleToY(yaw_angle);

            // Calculate the velocity in world coordinates
            velocity_wc = velocity_wc + delta_t * acceleration_wc;

            // Calculate the position in world coordinates
            position_wc = position_wc + delta_t * velocity_wc;

            // Move camera ?

            // Calculate the car angular velocity
            angular_velocity = angular_velocity + delta_t * angular_acceleration;

            // Update the angle from the car angular velocity
            yaw_angle = yaw_angle + delta_t * angular_velocity;

            // Calculate the wheel rotation rate (Wheels RPM)
            rpm_wheel = velocity.Length() / wheel_radius;


            // ------- Logger --------
            stats.Acceleration = acceleration_wc.Length();
            stats.Velocity = velocity_wc.Length() * 3.6f;
            stats.Gear = gear;

            sw.WriteLine("-------------------------------");
            sw.WriteLine("Angle: " + this.yaw_angle.ToString() + " SideSlip: " + side_slip_angle.ToString());
            sw.WriteLine("RPM: " + this.rpm_engine.ToString() + " Lookup Torque: " + LookupTorqueCurve(rpm_engine).ToString());
            sw.WriteLine("Gear: " + this.gear.ToString());
            sw.WriteLine("Ftraction: " + this.Ftraction.Length().ToString() + " Throttle: " + throttle.ToString() + " Brake: " + brake.ToString());
            sw.WriteLine("Fdrag: " + this.Fdrag.Length().ToString() + " Frr: " + this.Frr.Length().ToString());
            sw.WriteLine("Flong: " + this.Flong.ToString());
            sw.WriteLine("WC Acceleration: " + this.acceleration_wc.Length().ToString());
            sw.WriteLine("WC Velocity: " + this.velocity_wc.Length().ToString());
        }

        // Drag Force
        public Vector GetDragForce()
        {
            return -Cdrag * velocity * velocity.Length();
        }

        // Rolling Resistance Force
        public Vector GetRollingResistanceForce()
        {
            return -Crr * velocity;
        }

        // Braking Force
        public Vector GetBrakingForce()
        {
            return -new Vector(Cb, 0, 0);
        }

        // - Engine Force
        public Vector GetTractionForce()
        {
            return new Vector((throttle * torque_wheel) / wheel_radius, 0, 0);
        }

        public float GetWheelTorque()
        {
            return torque_engine * gears_ratio[gear] * Xd * n;
        }

        public float GetEngineTorque()
        {
            return LookupTorqueCurve(rpm_engine);
        }

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
                    return (torque_curve[i - 1, 1] + torque_curve[i, 1]) / 2;
                }

                // Si es el ultimo, devuelvo el tope de corte
                if (i == (len - 1))
                    return torque_curve[len - 1, 1];
            }

            return 0;
        }

        // Engine RPM
        public float GetEngineTurnOverRate()
        {
            float temp = FastMath.Abs((velocity.X * 60 * gears_ratio[gear] * Xd) / (2 * FastMath.PI * wheel_radius));

            if (temp > rpm_red_line)
                return rpm_red_line;

            if (temp < 1000)
                return 1000;

            return temp;
        }

        // Wheel angular velocity
        public float GetWheelAngularVelocity()
        {
            return 2 * FastMath.PI * rpm_engine / 60 * gears_ratio[gear] * Xd;
        }

        // ------------------------------------

        public void AutomaticTransmission()
        {
            if (velocity.Length() > max_speed_per_gear[gear])
            {
                if (gear == 6)
                {
                    return;
                }

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

        // ----------- Cornering --------------

        public float GetCarSideSlipAngle()
        {
            if (FastMath.Abs(velocity.X) < 1)
                return 0;

            if (velocity.X == 0)
                return 0;

            return FastMath.Atan(velocity.Z / FastMath.Abs(velocity.X));
        }

        public float GetFrontWheelSlipAngle()
        {
            if (FastMath.Abs(velocity.X) < 1)
                return 0;

            return FastMath.Atan((velocity.Z + (angular_velocity * b)) / FastMath.Abs(velocity.X)) - steering_angle * Math.Sign(velocity.X);
        }

        public float GetRearWheelSlipAngle()
        {
            if (FastMath.Abs(velocity.X) < 1)
                return 0;

            return FastMath.Atan((velocity.Z - (angular_velocity * c)) / FastMath.Abs(velocity.X));
        }

        public float GetFrontLateralForce()
        {
            float temp = Ca_front * wheel_front_slip_angle;

            /*
            if (FastMath.Abs(max_friction) < FastMath.Abs(temp))
                temp = max_friction;
            */
            temp = FastMath.Min(max_friction, temp);
            temp = FastMath.Max(-max_friction, temp);

            temp *= weight * 0.5f;
            //temp *= load;

            //temp *= 0.5f;

            return temp;
        }

        public float GetRearLateralForce()
        {
            float temp = Ca_rear * wheel_rear_slip_angle;
            /*
            if (FastMath.Abs(max_friction) < FastMath.Abs(temp))
                temp = max_friction;
            */
            temp = FastMath.Min(max_friction, temp);
            temp = FastMath.Max(-max_friction, temp);

            temp *= weight * 0.5f;
            //temp *= load; 

            //temp *= 0.5f;

            return temp;
        }

        public float GetCarTorque()
        {
            return FastMath.Cos(steering_angle) * Flat_front.Z * b - Flat_rear.Z * c;
        }

        // ------------------------------------

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

        // Collisions

        public void CrashWithObject()
        {
            velocity_wc = new Vector(0, 0, 0);
        }

        // Rendering

        public void Render()
        {
            // Set the new position
            mesh.Position = this.position_wc.ToDirectXVector();

            // Set the rotation
            mesh.Rotation = new Vector3(0, FastMath.PI + this.yaw_angle, 0);

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
