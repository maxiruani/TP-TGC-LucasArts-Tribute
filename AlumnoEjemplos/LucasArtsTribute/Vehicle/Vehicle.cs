using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;
using Microsoft.DirectX.DirectInput;
using TgcViewer;
using TgcViewer.Utils.TgcGeometry;
using TgcViewer.Utils.TgcSceneLoader;
using TgcViewer.Utils.Input;

using AlumnoEjemplos.LucasArtsTribute.Utils;
using AlumnoEjemplos.LucasArtsTribute.Components;

namespace AlumnoEjemplos.LucasArtsTribute.VehicleModel
{

    public static class MathHelper
    {
        /*
         * Constantes fisicas 
         */
        public static float PI = 3.141592654f;
        public static float g = 9.81f;              // Aceleracion de la gravedad
        public static float Mu = 1.0f;				// Coeficiente de friccion (1.0 para neumaticos de calle, 1.5 para neumaticos de competicion)
        public static float Rho = 1.29f;			// Densidad del aire

        /*
         * Convertir de grados a radianes
         */
        public static float Rad(float deg)
        {
            return (deg * PI) / 180.0f;
        }

        /*
         * Convertir de radianes a grados
         */
        public static float Deg(float rad)
        {
            return (rad * 180.0f) / PI;
        }

        /*
         * Retorna X trasladado en el angulo a 
         */
        public static float TranslateX(float x, float a, float f)
        {
            return x + (f * FastMath.Sin(a));
        }

        /*
         * Retorna Z trasladado en el angulo a 
         */
        public static float TranslateZ(float z, float a, float f)
        {
            return z + (f * FastMath.Cos(a));
        }

        /*
         * Retorna el angulo wrapeado
         */
        public static float WrapValue(float a)																					// returns angle wrapvalue
        {
            if (a < 0.0f) { return (2.0f * MathHelper.PI) + a; }
            if (a > (2.0f * MathHelper.PI)) { return a - (2.0f * MathHelper.PI); }
            return a;
        }

        /*
         * Retorna el valor interpolado
         */
        public static float CurveValue(float b, float a, float v, float delta_t)																
        {
            return a + ((b - a) * ((FastMath.Pow(v, delta_t)) - FastMath.Pow((v - 1.0f), delta_t)) / FastMath.Pow(v, delta_t));
        }

        /*
         * Retorna el angulo interpolado
         */
        public static float CurveAngle(float b, float a, float v, float delta_t)
        {
            float dif = 0.0f;
            float tmp = WrapValue(a - b);

            if (tmp <= MathHelper.PI) { dif = -1.0f * tmp; } else { dif = (2.0f * MathHelper.PI) - tmp; }
            return WrapValue(a + (dif * ((FastMath.Pow(v, delta_t) - FastMath.Pow(v - 1.0f, delta_t)) / FastMath.Pow(v, delta_t))));
        }

        /*
         * Retorna la distancia entre dos vectores
         */
        public static float Distance(Vector3 a, Vector3 b)
        {
            return FastMath.Sqrt(FastMath.Pow2(a.X - b.X) + FastMath.Pow2(a.Y - b.Y) + FastMath.Pow2(a.Z - b.Z));
        }

        /*
         * Retorna el angulo entre dos vectores
         */
        public static float Angle(Vector3 a, Vector3 b)
        {
            return FastMath.Acos(Vector3.Dot(a, b) / (a.Length() * b.Length()));
        }
    }

    public class EngineProperties
    {
        public float torque;				// engine torque
        public float rpm;					// engine rpm (revolutions per minute)
    }

    public class VehicleProperties
    {
        public string name;					// name of the vehicle
        public string bodyObj;				// path of the body object
        public string tyreObj;				// path of the tyre object
        public string bodyTex;				// path of the body texture
        public string tyreTex;				// path of the tyre texture

        /*
	    string engineSnd;				    // path of the engine sound
	    string hornSnd;					    // path of the horn sound
	    string brakeSnd;				    // path of the brake sound
        */

        public float mass;					// vehicle mass
        public float wheelMass;				// vehicle wheel mass
        public float drag;					// coefficient of frition
        public float frontArea;				// frontal area
        public float bodyLength;			// length of vehicle body
        public float bodyWidth;				// width of vehicle body
        public float bodyHeight;			// height of vehicle body
        public float wheelRadius;			// radius of the vehicle wheel
	    
        
        public float groundHeight;				    // height between vehicle and ground
	    /*
        float springLength;				    // original length of the suspension spring			
	    float springConst;				    // suspension spring constant
	    */

        public float brakeTorque;			// torque applied to lock wheels
        public float[] gearRatio;			// stores the gear 1 to 6 ratios (0 is reverse)
        public float diffRatio;				// stores the differential ratio
	    public float transEff;					    // stores the transmission efficiency

        public Vector3 cog;				    // vehicle centre of gravity (z axis offset)
        public Vector3 fl;					// default position of front left wheel
        public Vector3 fr;					// default position of front right wheel
        public Vector3 bl;					// default position of back left wheel
        public Vector3 br;					// default position of back right wheel
        public Vector3 bScl;				// scale factors for the vehicle body
        public Vector3 wScl;				// scale factors for the vehicle tyre

        public EngineProperties[] engine;	// rpm/torque lookup curve container

        public VehicleProperties()
        {
        }
    }

    public class Vehicle
    {
        /*
         * Variables de renderizado
         */

        // Meshes
        public TgcMesh body;			// pointer to car body
        public TgcMesh flw;			    // pointer to car front left wheel
        public TgcMesh frw;				// pointer to car front right wheel	
        public TgcMesh blw;				// pointer to car bakc left wheel
        public TgcMesh brw;				// pointer to car back right wheel

        /*
         * Variables matematicas
         */

        float wAng;						// stores the steering (front) wheels angle
        float oWang;					// stores the old steering (front) wheels angle
        float nxWeightAng;				// stores the new x axis weight transfer angle (after curve)
        float oxWeightAng;				// stores the old x axis weight transfer angle (before curve)
        float nzWeightAng;				// stores the new z axis weight transfer angle (after curve)
        float ozWeightAng;				// stores the old z axis weight transfer angle (before curve)

        Vector3 fPos;					// stores vehicle front ground position
        Vector3 bPos;					// stores vehicle back ground position
        Vector3 lPos;					// stores vehicle left ground position
        Vector3 rPos;					// stores vehicle right ground position
        Vector3 bDir;					// stores the vehicle body direction
        Vector3 bNDir;					// stores the vehicle body normal direction
        Vector3 vDir;					// stores the vehicle velocity direction
        Vector3 fwvDir;					// stores the vehicle front wheel velocity direction
        Vector3 flwDir;					// stores the front left wheel steer direction
        Vector3 frwDir;					// stores the front right wheel steer direction
        Vector3 flwNDir;				// stores the front left wheel normal direction
        Vector3 frwNDir;				// stores the front right wheel normal direction
        Vector3 blwNDir;				// stores the back left wheel normal direction
        Vector3 brwNDir;				// stores the back right wheel normal direction

        /*
         * Variables fisicas
         */
        // Propiedades
        VehicleProperties make;			// stores the unique properties of the car

        // Escalares
        int s_gear;						// stores the current vehicle gear (1 to 6 or 0 for reverse)
        int s_ogear;					// stores the old vehicle gear
        int s_dir;						// stores the direction the vehicle is moving in (1=front,0=still,-1=back)
        float s_lsRadius;				// stores the low speed radius of curvature
        float s_hsRadius;				// stores the high speed radius of curvature
        float s_centripetalForce;		// stores the centripetal cornering force
        float s_frontLateralForce;		// stores the lateral force from the front wheels
        float s_rearLateralForce;		// stores the lateral force from the rear wheels
        float s_frontCornerStiff;		// stores the cornering stiffness of the front wheels
        float s_rearCornerStiff;		// stores the cornering stiffness of the rear wheels
        float s_omega;					// vehicle angular velocity at low speed
        float s_omega2;					// vehicle angular velocity at high speed
        float s_frontAlpha;				// vehicle front wheel sideslip angle
        float s_rearAlpha;				// vehicle rear wheel sideslip angle
        float s_beta;					// vehicle body sideslip angle
        float s_slipRatio;				// stores the slip ratio of the vehicle
        float s_netTorque;				// sum of torque acting on the rear axle
        float s_rpm;					// revolutions per minute of the engine
        float s_fwAngVelocity;			// stores the angular velocity of the front wheels
        float s_bwAngVelocity;			// stores the angular velocity of the back wheels
        float s_bwAngAcceleration;		// stores the angular acceleration of the back wheels
        float s_wheelIntertia;			// stores the intertia of the wheel cylinder
        float s_maxTorque;				// stores the maximum torque tapped from the engine
        float s_engineForce;			// torque supplied by the vehicle engine
        float s_absoluteForce;			// result of forces acting on vehicle
        float s_weightForce;			// force acting on vehicle due to ground inclinations
        float s_throttleAmount;			// percentage of engine torque tapped by the throttle position
        float s_brakeAmount;			// percentage of force on the wheels to break
        float s_dragConstant;			// constant coefficient of drag
        float s_speed;					// speed of the vehicle
        float s_rollResistanceConstant;	// constant coefficient of roll resistance
        float s_brakeForce;				// constant coefficient of breaking
        float s_blwMaxTraction;			// stores the back left wheel maximum traction force
        float s_brwMaxTraction;			// stores the back right wheel maximum traction force
        float s_flwMaxTraction;			// stores the front left wheel maximum traction force
        float s_frwMaxTraction;			// stores the front right wheel maximum traction force
        float s_blwMaxTorque;			// stores the back left wheel torque opposing the engine force
        float s_brwMaxTorque;			// stores the back right wheel torque opposing the engine force

        float s_vehicleWeight;			// weight of the entire vehicle
        float s_wheelWeight;			// weight of one single wheel
        float s_flwWeight;				// weight of front left wheel
        float s_frwWeight;				// weight of front right wheel
        float s_blwWeight;				// weight of back left wheel
        float s_brwWeight;				// weight of back right wheel

        float s_inertia;				// inertia of the vehicle
        float s_bodyCurveX;				// stores the body x axis curve angle
        float s_bodyCurveY;				// stores the body y axis curve angle 
        float s_bodyCurveZ;				// stores the body z axis curve angle
        float s_flwCurveAng;			// stores the front left wheel y axis curve anlge
        float s_frwCurveAng;			// stores the front right wheel y axis curve angle

        /*
         * Vectores
         */

        Vector3 v_brakingForce;			// force applied on wheels to stop motion
        Vector3 v_tractionForce;		// force delivered to wheels by the engine
        Vector3 v_dragForce;			// air resistance on the car body
        Vector3 v_rollResistanceForce;	// friction between wheel and road, axle etc
        Vector3 v_longitudinalForce;	// force in the direction of the car body	
        Vector3 v_velocity;				// velocity Vector3 of the vehicle
        Vector3 v_acceleration;			// acceleration due to gravity on the vehicle
        Vector3 v_position;				// position of the vehicle
        Vector3 v_oldPosition;			// old position of the vehicle
        Vector3 v_sCOG;					// stores the source centre of gravity
        Vector3 v_rCOG;					// stores the result centre of gravity
        Vector3 v_turnFriction;			// force opposing motion of vehicle when turning
        Vector3 v_flwSPos;				// stores the source position for the front left wheel
        Vector3 v_flwRPos;				// stores the result position for the front left wheel
        Vector3 v_frwSPos;				// stores the source position for the front right wheel
        Vector3 v_frwRPos;				// stores the result position for the front right wheel
        Vector3 v_blwSPos;				// stores the source position for the back left wheel
        Vector3 v_blwRPos;				// stores the result position for the back left wheel
        Vector3 v_brwSPos;				// stores the source position for the back right wheel
        Vector3 v_brwRPos;				// stores the result position for the back right wheel

        /*
		 * Variables varias
         */

		// bool				iActive;					// determines if illustration is active
		bool				breaking;					// determines if vehicle is breaking
		bool				freeMoving;					// determines if vehicle not under engine force
		bool				gKey;						// to enforce one touch gear key pressing
		// char				camTyp;						// stores the vehicle camera type

        public float delta_t;                           // Elapsed time

        StreamWriter sw;

        public Vehicle(String path)
        {
            //sw = new StreamWriter("D:\\TGC-Logs\\log.txt");
            //sw.WriteLine("Instance Car");

            // Seteo las propiedades del auto que se obtuvieron del archivo de configuracion.
            bool isOk = SetConfigData(path);

            if (!isOk)
                make = null;

            ResetVehicle();		// Inicializo el vehiculo
            SetupVehicle();	    // Seteo el vehiculo
            ResetVariables();	// Inicializo todas las variables
        }

        private bool SetConfigData(String path)
        {
            ConfigurationManager config = new ConfigurationManager();

            try
            {
                if (!config.ReadConfigFile(path))
                    return false;

                make = new VehicleProperties();

                make.name = config.GetValue("name");
                make.brakeTorque = config.GetFloatValue("brakeTorque");
                make.mass = config.GetFloatValue("mass");
                make.wheelMass = config.GetFloatValue("wheelMass");
                make.drag = config.GetFloatValue("drag");
                make.frontArea = config.GetFloatValue("frontArea");
                make.bodyLength = config.GetFloatValue("bodyLength");
                make.bodyWidth = config.GetFloatValue("bodyWidth");
                make.bodyHeight = config.GetFloatValue("bodyHeight");
                make.cog.X = config.GetFloatValue("cog.x");
                make.cog.Y = config.GetFloatValue("cog.y");
                make.cog.Z = config.GetFloatValue("cog.z");
                make.wheelRadius = config.GetFloatValue("wheelRadius");
                make.groundHeight = config.GetFloatValue("groundHeight");

                //make.springLength = config.GetFloatValue("springLength");
                //make.springConst = config.GetFloatValue("springConst");

                make.transEff = config.GetFloatValue("transEff");
                make.diffRatio = config.GetFloatValue("diffRatio");

                make.gearRatio = new float[7];
                make.gearRatio[0] = config.GetFloatValue("gearRatio[0]");
                make.gearRatio[1] = config.GetFloatValue("gearRatio[1]");
                make.gearRatio[2] = config.GetFloatValue("gearRatio[2]");
                make.gearRatio[3] = config.GetFloatValue("gearRatio[3]");
                make.gearRatio[4] = config.GetFloatValue("gearRatio[4]");
                make.gearRatio[5] = config.GetFloatValue("gearRatio[5]");
                make.gearRatio[6] = config.GetFloatValue("gearRatio[6]");

                // Posiciones default de las ruedas
                make.fl.X = config.GetFloatValue("flpos.x"); make.fl.Y = config.GetFloatValue("flpos.y"); make.fl.Z = config.GetFloatValue("flpos.z");
                make.fr.X = config.GetFloatValue("frpos.x"); make.fr.Y = config.GetFloatValue("frpos.y"); make.fr.Z = config.GetFloatValue("frpos.z");
                make.bl.X = config.GetFloatValue("blpos.x"); make.bl.Y = config.GetFloatValue("blpos.y"); make.bl.Z = config.GetFloatValue("blpos.z");
                make.br.X = config.GetFloatValue("brpos.x"); make.br.Y = config.GetFloatValue("brpos.y"); make.br.Z = config.GetFloatValue("brpos.z");

                // Escalas del cuerpo y ruedas del vehiculo
                make.bScl.X = config.GetFloatValue("bScl.x"); make.bScl.Y = config.GetFloatValue("bScl.y"); make.bScl.Z = config.GetFloatValue("bScl.z");
                make.wScl.X = config.GetFloatValue("wScl.x"); make.wScl.Y = config.GetFloatValue("wScl.y"); make.wScl.Z = config.GetFloatValue("wScl.z");
                
                // Valores de curva interpolada torque/rpm
                make.engine = new EngineProperties[6];

                for (int i = 0; i < 6; i++)
                    make.engine[i] = new EngineProperties();

                make.engine[0].rpm = config.GetFloatValue("engine[0].rpm"); make.engine[0].torque = config.GetFloatValue("engine[0].trq");
                make.engine[1].rpm = config.GetFloatValue("engine[1].rpm"); make.engine[1].torque = config.GetFloatValue("engine[1].trq");
                make.engine[2].rpm = config.GetFloatValue("engine[2].rpm"); make.engine[2].torque = config.GetFloatValue("engine[2].trq");
                make.engine[3].rpm = config.GetFloatValue("engine[3].rpm"); make.engine[3].torque = config.GetFloatValue("engine[3].trq");
                make.engine[4].rpm = config.GetFloatValue("engine[4].rpm"); make.engine[4].torque = config.GetFloatValue("engine[4].trq");
                make.engine[5].rpm = config.GetFloatValue("engine[5].rpm"); make.engine[5].torque = config.GetFloatValue("engine[5].trq");

                // Seteo los paths de los meshes y texturas
                make.bodyObj = config.GetValue("body.mesh");
                make.bodyTex = config.GetValue("body.texture");
                make.tyreObj = config.GetValue("tyre.mesh");
                make.tyreTex = config.GetValue("tyre.texture");

            }
            catch (Exception e)
            {
                return false;
            }

            return true;
        }

        private void SetupVehicle()
        {
            // Cargo el mesh del cuerpo del vehiculo
            TgcSceneLoader loader = new TgcSceneLoader();
            TgcScene scene = loader.loadSceneFromFile(GuiController.Instance.AlumnoEjemplosMediaDir + make.bodyObj);
            body = scene.Meshes[0];

            // Cargo los meshes de las ruedas del vehiculo
            scene = loader.loadSceneFromFile(GuiController.Instance.AlumnoEjemplosMediaDir + make.tyreObj);
            flw = scene.Meshes[0];
            flw.Name = "flw";
            frw = scene.Meshes[0].clone("frw");
            blw = scene.Meshes[0].clone("blw");
            brw = scene.Meshes[0].clone("brw");

            // Determino el centro de gravedad
            v_sCOG = new Vector3(body.Position.X + make.cog.X, body.Position.Y + make.cog.Y, body.Position.Z + make.cog.Z);
            v_sCOG = body.TransformCoord(v_sCOG);

            // Seteo las posiciones de las ruedas dependiendo de la posicion del cuerpo
            v_flwSPos = new Vector3(body.Position.X + make.fl.X, body.Position.Y + make.fl.Y, body.Position.Z + make.fl.Z); // Frontal Izquierda
            v_frwSPos = new Vector3(body.Position.X + make.fr.X, body.Position.Y + make.fr.Y, body.Position.Z + make.fr.Z); // Frontal Derecha
            v_blwSPos = new Vector3(body.Position.X + make.bl.X, body.Position.Y + make.bl.Y, body.Position.Z + make.bl.Z); // Trasera Izquierda
            v_brwSPos = new Vector3(body.Position.X + make.br.X, body.Position.Y + make.br.Y, body.Position.Z + make.br.Z); // Trasera Derecha

            body.Position = new Vector3(5, 0, 5);       // Posicion inicial
            this.TransformWheelPos();                   // Trasnformo las posiciones de las ruedas a las coordenadas del cuerpo del vehiculo

            // Seteo el tamaño del vehiculo y de las rueadas
            body.Scale = new Vector3(make.bScl.X, make.bScl.Y, make.bScl.Z);
            
            flw.Scale = new Vector3(make.wScl.X, make.wScl.Y, make.wScl.Z);
            frw.Scale = new Vector3(make.wScl.X, make.wScl.Y, make.wScl.Z);
            blw.Scale = new Vector3(make.wScl.X, make.wScl.Y, make.wScl.Z); 
            brw.Scale = new Vector3(make.wScl.X, make.wScl.Y, make.wScl.Z);
        }

        /*
         * Transforma el centro de gravedad (basado en la traslacion y la rotacion)
         */
        public void TransformCOG()
        { 
            v_rCOG = body.TransformCoord(v_sCOG);
        }

        /*
         * Transforma las posiciones de las ruedas a las coordenadas del cuerpo del vehiculo
         */
        public void TransformWheelPos()
        {
            v_flwRPos = body.TransformCoord(v_flwSPos);
            v_frwRPos = body.TransformCoord(v_frwSPos);
            v_blwRPos = body.TransformCoord(v_blwSPos);
            v_brwRPos = body.TransformCoord(v_brwSPos);
        }

        /*
         * Calcular las coordenadas del frente, trasero, derecha a e izquierda del auto.
         */
        void CalcVehicleCoord()
        {
            // get car front ground coordinates
            fPos.X = MathHelper.TranslateX(body.Position.X, body.Rotation.Y, make.fl.Z);
            fPos.Z = MathHelper.TranslateZ(body.Position.Z, body.Rotation.Y, make.fl.Z);
            fPos.Y = 0;
            //fPos.Y = land->getElandHeight(fPos.X, fPos.Z, 0);

            // get car back ground coordinates
            bPos.X = MathHelper.TranslateX(body.Position.X, body.Rotation.Y, make.bl.Z);
            bPos.Z = MathHelper.TranslateZ(body.Position.Z, body.Rotation.Y, make.bl.Z);
            bPos.Y = 0;
            //bPos.Y = land->getElandHeight(bPos.X, bPos.Z, 0);

            // get car left ground coordinates
            lPos.X = MathHelper.TranslateX(body.Position.X, body.Rotation.Y + (MathHelper.PI / 2.0f), make.fl.X);
            lPos.Z = MathHelper.TranslateZ(body.Position.Z, body.Rotation.Y + (MathHelper.PI / 2.0f), make.fl.X);
            lPos.Y = 0;
            //lPos.Y = land->getElandHeight(lPos.X, lPos.Z, 0);

            // get car right ground coordinates
            rPos.X = MathHelper.TranslateX(body.Position.X, body.Rotation.Y + (MathHelper.PI / 2.0f), make.fr.Z);
            rPos.Z = MathHelper.TranslateZ(body.Position.Z, body.Rotation.Y + (MathHelper.PI / 2.0f), make.fr.Z);
            rPos.Y = 0;
            //rPos.Y = land->getElandHeight(rPos.X, rPos.Z, 0);
        }

        /*
         * Calcular la fuerza peso del cuerpo
         */
        public void CalcWeightForce()
        {
            CalcVehicleCoord();						        // Calcular las coordenadas del vehiculo
            s_vehicleWeight = MathHelper.g * make.mass;     // Calcular el peso del vehiculo
            // s_weightForce = s_vehicleWeight;
            s_weightForce = 0;
        }

        /*
         * Calcular la fuerza peso de las ruedas
         */
        public void CalcWheelWeight()
        {
            // Construyo el plano al nivel de las ruedas y su normal
			Plane wPlane = Plane.FromPoints(v_flwRPos, v_frwRPos, v_blwRPos);
			Vector3 wPlaneNormal = new Vector3(wPlane.A, wPlane.B, wPlane.C);

            // Creo un rayo desde su inicio a final
			Vector3 r1 = new Vector3(v_rCOG.X, v_rCOG.Y, v_rCOG.Z);
			Vector3 r2 = new Vector3(v_rCOG.X, v_rCOG.Y - 100.0f, v_rCOG.Z);
			Vector3 ray = r2 - r1;

            // Chequeo que el rayo cruce el plano
			if ( ((wPlane.Dot(r1) < 0.0f) && (wPlane.Dot(r2) > 0.0f)) || ((wPlane.Dot(r1) > 0.0f ) && (wPlane.Dot(r2) < 0.0f)) )
			{
				if(Vector3.Dot(wPlaneNormal, ray) != 0)
				{
					// Calculo la interseccion de las coordenadas
					float amt = -(wPlane.Dot(r1)) / Vector3.Dot(wPlaneNormal, ray);
					Vector3 i = r1 + (ray * amt);

                    // Chequeo si la interseccion coordina sin las ruedas
					Vector3 IA = new Vector3(v_flwRPos.X - i.X, v_flwRPos.Y - i.Y, v_flwRPos.Z - i.Z);
					Vector3 IB = new Vector3(v_frwRPos.X - i.X, v_frwRPos.Y - i.Y, v_frwRPos.Z - i.Z);
					Vector3 IC = new Vector3(v_blwRPos.X - i.X, v_blwRPos.Y - i.Y, v_blwRPos.Z - i.Z);
					Vector3 ID = new Vector3(v_brwRPos.X - i.X, v_brwRPos.Y - i.Y, v_brwRPos.Z - i.Z);
                    
                    // El angulo interior debe ser aprox 360 deg/2pi rad
					float angTotal = FastMath.Acos(Vector3.Dot(IA, IB) / (IA.Length() * IB.Length())) + FastMath.Acos(Vector3.Dot(IB, ID) / (IB.Length() * ID.Length())) + FastMath.Acos(Vector3.Dot(ID, IC) / (ID.Length() * IC.Length())) + FastMath.Acos(Vector3.Dot(IC, IA) / (IC.Length() * IA.Length()));

					if(angTotal > 6.28f)
					{ 
                        // Calcula las distancias entreruedas
                        float dist_i_flw = MathHelper.Distance(i, v_flwRPos);
                        float dist_i_frw = MathHelper.Distance(i, v_frwRPos);
                        float dist_i_blw = MathHelper.Distance(i, v_blwRPos);
                        float dist_i_brw = MathHelper.Distance(i, v_brwRPos);
                        float dist_i_org = MathHelper.Distance(i, r1);
                        float totalDist = dist_i_flw + dist_i_frw + dist_i_blw + dist_i_brw;

                        // Calculo los pesos en las ruedas estaticos (sin influencias)
                        s_flwWeight = s_wheelWeight + ((dist_i_frw + dist_i_blw + dist_i_brw) / totalDist) * s_vehicleWeight;
                        s_frwWeight = s_wheelWeight + ((dist_i_flw + dist_i_blw + dist_i_brw) / totalDist) * s_vehicleWeight;
                        s_blwWeight = s_wheelWeight + ((dist_i_flw + dist_i_frw + dist_i_brw) / totalDist) * s_vehicleWeight;
                        s_brwWeight = s_wheelWeight + ((dist_i_flw + dist_i_frw + dist_i_blw) / totalDist) * s_vehicleWeight;

                        // Inicializo las variables de aceleracion y peso
                        float lngAccel = 0.0f, lngAccelWeight = 0.0f;
                        float latAccel = 0.0f, latAccelWeight = 0.0f;

                        // Determino la aceleracion longitudinal a lo largo del cuerpo del vehiculo
                        if (freeMoving) 
                            lngAccel = - v_acceleration.Length();
                        else 
                            lngAccel = v_acceleration.Length();

                        lngAccelWeight = ((dist_i_org / make.bodyLength) * make.mass * lngAccel);

                        // Determino la aceleracion latera, perpendicular al cuerpo del vehiculo
                        if (wAng > 0.0f) latAccel = -s_centripetalForce / make.mass;
                        if (wAng < 0.0f) latAccel = s_centripetalForce / make.mass;
                        latAccelWeight = ((dist_i_org / make.bodyWidth) * make.mass * latAccel);

                        if (s_dir > -1)
                        {
                            // Calculo las fuerzas dinamicas de peso de las ruedas influenciadas por la aceleracion longitudinal
                            s_flwWeight = s_flwWeight - ((dist_i_frw / (dist_i_flw + dist_i_frw)) * lngAccelWeight);
                            s_frwWeight = s_frwWeight - ((dist_i_flw / (dist_i_flw + dist_i_frw)) * lngAccelWeight);
                            s_blwWeight = s_blwWeight + ((dist_i_brw / (dist_i_blw + dist_i_brw)) * lngAccelWeight);
                            s_brwWeight = s_brwWeight + ((dist_i_blw / (dist_i_blw + dist_i_brw)) * lngAccelWeight);

                            // Calculo las fuerzas dinamicas de peso de las ruedas influenciadas por la aceleracion lateral
                            s_flwWeight = s_flwWeight - ((dist_i_blw / (dist_i_blw + dist_i_flw)) * latAccelWeight);
                            s_frwWeight = s_frwWeight + ((dist_i_brw / (dist_i_brw + dist_i_frw)) * latAccelWeight);
                            s_blwWeight = s_blwWeight - ((dist_i_flw / (dist_i_flw + dist_i_blw)) * latAccelWeight);
                            s_brwWeight = s_brwWeight + ((dist_i_frw / (dist_i_frw + dist_i_brw)) * latAccelWeight);
                        }
                        else
                        {
                            // Calculo las fuerzas dinamicas de peso de las ruedas influenciadas por la aceleracion longitudinal
                            s_flwWeight = s_flwWeight + ((dist_i_frw / (dist_i_flw + dist_i_frw)) * lngAccelWeight);
                            s_frwWeight = s_frwWeight + ((dist_i_flw / (dist_i_flw + dist_i_frw)) * lngAccelWeight);
                            s_blwWeight = s_blwWeight - ((dist_i_brw / (dist_i_blw + dist_i_brw)) * lngAccelWeight);
                            s_brwWeight = s_brwWeight - ((dist_i_blw / (dist_i_blw + dist_i_brw)) * lngAccelWeight);

                            // Calculo las fuerzas dinamicas de peso de las ruedas influenciadas por la aceleracion lateral
                            s_flwWeight = s_flwWeight + ((dist_i_blw / (dist_i_blw + dist_i_flw)) * latAccelWeight);
                            s_frwWeight = s_frwWeight - ((dist_i_brw / (dist_i_brw + dist_i_frw)) * latAccelWeight);
                            s_blwWeight = s_blwWeight + ((dist_i_flw / (dist_i_flw + dist_i_blw)) * latAccelWeight);
                            s_brwWeight = s_brwWeight - ((dist_i_frw / (dist_i_frw + dist_i_brw)) * latAccelWeight);
                        }

                        // Calculo la diferencia de peso del vehiculo
                        oxWeightAng = ((s_blwWeight + s_brwWeight) - (s_flwWeight + s_frwWeight)) * 0.00002f;
                        ozWeightAng = ((s_frwWeight + s_brwWeight) - (s_flwWeight + s_blwWeight)) * 0.0000009f;

                        // Limite de diferencia de peso
                        if (oxWeightAng > 0.2) oxWeightAng = 0.2f; if (oxWeightAng < -0.2) oxWeightAng = -0.2f;
                        if (ozWeightAng > 0.3) ozWeightAng = 0.3f; if (ozWeightAng < -0.3) ozWeightAng = -0.3f;

                        // Aplico la diferencia de peso gradualmente
                        nxWeightAng = MathHelper.CurveAngle(oxWeightAng, nxWeightAng, 1.3f, delta_t);
                        nzWeightAng = MathHelper.CurveAngle(ozWeightAng, nzWeightAng, 1.1f, delta_t);

                    }
                }
            }
        }

        /*
         * Calcular las direcciones de los vectores. (Cuerpo y ruedas) 
         */
        public void CalcDirVect()
        {
            float x = 0.0f, y = 0.0f, z = 0.0f, s = 0.0f;
            float h = body.Position.Y /*- land->getElandHeight(body.Position.X, body.Position.Z, 0)*/;
            Vector3 nPos, oPos;

            // ---        ---
            // --- Cuerpo ---
            // ---        ---

            // --- Obtener la direccion del cuerpo

            // Obtener la vieja y nueva posicion
            oPos = body.Position;
            x = MathHelper.TranslateX(body.Position.X, body.Rotation.Y, 3.0f);
            z = MathHelper.TranslateZ(body.Position.Z, body.Rotation.Y, 3.0f);
            y = h /*+ land->getElandHeight(x, z, 0)*/;
            nPos = new Vector3(x, y, z);
            bDir = nPos - oPos; 

            // Calculo la direccion del vector
            bDir.Normalize();

            // --- Obtener la direccion normal del cuerpo

            // Determino angulo de giro
            if (wAng > 0.0f) oWang = -1.5714f;
            if (wAng < 0.0f) oWang = +1.5714f;

            nPos = new Vector3(MathHelper.TranslateX(body.Position.X, body.Rotation.Y + oWang, 2.0f), body.Position.Y, MathHelper.TranslateZ(body.Position.Z, body.Rotation.Y + oWang, 2.0f));
            bNDir = nPos - oPos;
            bNDir.Normalize();

            // --- Obtener el vector de velocidad del cuerpo

            // Velocidad que depende de la velocidad angular
            s = FastMath.Exp(FastMath.Abs(s_omega) / 100.0f);
            s_bodyCurveY = MathHelper.CurveValue(body.Rotation.Y, s_bodyCurveY, s, delta_t);

            // Obtener la nueva posicion del cuerpo
            x = MathHelper.TranslateX(body.Position.X, s_bodyCurveY, 3.0f);
            z = MathHelper.TranslateZ(body.Position.Z, s_bodyCurveY, 3.0f);
            y = h /*+ land->getElandHeight(x, z, 0)*/;
            nPos = new Vector3(x, y, z);
            // Calcular el vector de direccion del cuerpo
            vDir = nPos - oPos;
            vDir.Normalize();

            // ---                         ---
            // --- Rueda Frontal Izquierda ---
            // ---                         ---

            // --- Obtengo la direccion Normal de la Rueda Frontal Izquierda

            // Obtengo la vieja y nueva posicion de la Rueda Frontal Izquierda
            oPos = flw.Position;
            nPos = new Vector3(MathHelper.TranslateX(flw.Position.X, flw.Rotation.Y - 1.5714f, 30.0f), flw.Position.Y, MathHelper.TranslateZ(flw.Position.Z, flw.Rotation.Y - 1.5714f, 30.0f));
            flwNDir = nPos - oPos;
            // Calculo el vector direccion de la rueda
            flwNDir.Normalize();


            // --- Obtengo la direccion de la Rueda Frontal Izquierda

            // Obtengo la nueva posicion de la Rueda Frontal Izquierda
            nPos = new Vector3(MathHelper.TranslateX(flw.Position.X, flw.Rotation.Y, 5.0f), flw.Position.Y, MathHelper.TranslateZ(flw.Position.Z, flw.Rotation.Y, 5.0f));
            flwDir = nPos - oPos;
            // Calculo el vector direccion de la rueda
            flwDir.Normalize();							

            // --- Obtengo el vector direccion de velocidad de la Rueda Frontal Izquierda

            // Obtengo la nueva posicion del vector velocidad de la rueda
            s_flwCurveAng = MathHelper.CurveValue(flw.Rotation.Y, s_flwCurveAng, s, delta_t);
            x = MathHelper.TranslateX(flw.Position.X, s_flwCurveAng, 5.0f);
            z = MathHelper.TranslateZ(flw.Position.Z, s_flwCurveAng, 5.0f);
            y = flw.Position.Y;
            nPos = new Vector3(x, y, z);

            // Calcular el vector de velocidad de las ruedas
            fwvDir = nPos - oPos;
            fwvDir.Normalize();

            // ---                       ---
            // --- Rueda Frontal Derecha ---
            // ---                       ---

            // --- Obtengo la direccion Normal de la Rueda Frontal Derecha

            // Obtengo la vieja y nueva posicion de la Rueda Frontal Derecha
            oPos = frw.Position;
            nPos = new Vector3(MathHelper.TranslateX(frw.Position.X, frw.Rotation.Y + 1.5714f, 30.0f), frw.Position.Y, MathHelper.TranslateZ(frw.Position.Z, frw.Rotation.Y + 1.5714f, 30.0f));
            frwNDir = nPos - oPos;
            // Calculo el vector direccion de la rueda
            frwNDir.Normalize();

            // --- Obtengo la direccion de la Rueda Frontal Derecha

            // Obtengo la nueva posicion del vector velocidad de la rueda
            nPos = new Vector3(MathHelper.TranslateX(frw.Position.X, frw.Rotation.Y, 5.0f), frw.Position.Y, MathHelper.TranslateZ(frw.Position.Z, frw.Rotation.Y, 5.0f));
            frwDir = nPos - oPos;
            // Calculo el vector direccion de la rueda
            frwDir.Normalize();

            // --- Obtengo el vector direccion de velocidad de la Rueda Frontal Derecha

            // Obtengo la nueva posicion del vector velocidad de la rueda
            s_frwCurveAng = MathHelper.CurveValue(frw.Rotation.Y, s_frwCurveAng, s, delta_t);
            x = MathHelper.TranslateX(frw.Position.X, s_frwCurveAng, 5.0f);
            z = MathHelper.TranslateZ(frw.Position.Z, s_frwCurveAng, 5.0f);
            y = frw.Position.Y;
            nPos = new Vector3(x, y, z);

            // ---                         ---
            // --- Rueda Trasera Izquierda ---
            // ---                         ---

            // Obtengo la direccion de la Rueda Trasera Izquierda

            // Obtengo la vieja y nueva direccion
            oPos = blw.Position;
            nPos = new Vector3(MathHelper.TranslateX(blw.Position.X, blw.Rotation.Y - 1.5714f, 30.0f), blw.Position.Y, MathHelper.TranslateZ(blw.Position.Z, blw.Rotation.Y - 1.5714f, 30.0f));
            blwNDir = nPos - oPos;

            // Calculo el vector direccion de la rueda
            blwNDir.Normalize();

            // ---                       ---
            // --- Rueda Trasera Derecha ---
            // ---                       ---

            // Obtengo la direccion de la Rueda Trasera Derecha

            // Obtengo la vieja y nueva direccion
            oPos = brw.Position;
            nPos = new Vector3(MathHelper.TranslateX(brw.Position.X, brw.Rotation.Y + 1.5714f, 30.0f), brw.Position.Y, MathHelper.TranslateZ(brw.Position.Z, brw.Rotation.Y + 1.5714f, 30.0f));
            brwNDir = nPos - oPos; 

            // Calculo el vector direccion de la rueda
            brwNDir.Normalize();
        }

        /*
         * Calcular la velocidad angular del auto cuando esta girando
         */
        public void CalcAngVel()
        {
            s_omega = 0.0f;     // Inicializo la velocidad angular

            if (wAng > 0.0f)    // Girando hacia la derecha
            {
                float l = MathHelper.Distance(v_frwRPos, v_brwRPos);											// Distancia del eje entre las ruedas derechas
                float a = FastMath.Acos(Vector3.Dot(frwNDir, brwNDir) / (frwNDir.Length() * brwNDir.Length()));	// Angulo entre los vectores de direccion de las ruedas
                float d = FastMath.Exp(s_speed / 10.0f) - 1.0f;												    // Aumento exponencial radial
                s_lsRadius = l / FastMath.Sin(a);															    // Radio de curvatura a baja velocidad
                s_omega = +s_speed / (s_lsRadius + d);												            // Velocidad angular
            }

            if (wAng < 0.0f)    // Girando hacia la izquierda
            {
                float l = MathHelper.Distance(v_flwRPos, v_blwRPos);											// Distancia del eje entre las ruedas izquierdas
                float a = FastMath.Acos(Vector3.Dot(flwNDir, blwNDir) / (flwNDir.Length() * blwNDir.Length()));	// Angulo entre los vectores de direccion de las ruedas
                float d = FastMath.Exp(s_speed / 10.0f) - 1.0f;												    // Aumento exponencial radial
                s_lsRadius = l / FastMath.Sin(a);															    // Radio de curvatura a baja velocidad
                s_omega = -s_speed / (s_lsRadius + d);												            // Velocidad angular
            }
        }

        /*
         * Calcular la velocidad angular del auto cuando esta girando a alta velocidad
         */
        public void CalcAngVel2()
        {
            CalcVehicleCoord();																	    // Calcular coordenadas del vehiculo
            float b = MathHelper.Distance(v_rCOG, fPos);										    // Distancia entre el centro de gravedad y el frente del vechiculo
            float c = MathHelper.Distance(v_rCOG, bPos);										    // Distancia entre el centro de gravedad y la parte trasera del vehiculo
            float t = (FastMath.Cos(wAng) * s_frontLateralForce * b) + (s_rearLateralForce * c);    // Torque angular total
            float a = t / s_inertia;															    // Aceleracion angular
            s_omega2 = delta_t * a;												                    // Velocidad angular
        }

        /*
         * Setear la posicion de la Rueda Frontal Izquierda
         */
        public void PositionFrontLeftWheel()
        {
            // Seteo la posicion
            flw.Position = new Vector3(v_flwRPos.X, v_flwRPos.Y /*+ s_springNExt - s_oscExt*/, v_flwRPos.Z);

            // Seteo el angulo
            flw.rotateX((s_fwAngVelocity * delta_t) / (2.0f * MathHelper.PI));

            if (flw.Rotation.X > 6.2857f) 
                flw.rotateX(-6.2857f);

            flw.SetRotationY(body.Rotation.Y + wAng);
            //flw.SetRotationZ(body.Rotation.Z /*+ nzWeightAng*/); // TODO: Chequear esto.
        }

        /*
         * Setear la posicion de la Rueda Frontal Derecha
         */
        public void PositionFrontRightWheel()
        {
            // Seteo la posicion
            frw.Position = new Vector3(v_frwRPos.X, v_frwRPos.Y /*+ s_springNExt - s_oscExt*/, v_frwRPos.Z);

            // Seteo el angulo
            frw.rotateX((s_fwAngVelocity * delta_t) / (2.0f * MathHelper.PI));
            
            if (frw.Rotation.X > 6.2857f) 
                frw.rotateX(-6.2857f);

            frw.SetRotationY(body.Rotation.Y + wAng);
            //frw.SetRotationZ(body.Rotation.Z /*+ nzWeightAng*/); // TODO: Chequear esto.
        }

        /*
         * Setear la posicion de la Rueda Trasera Izquierda
         */
        public void PositionBackLeftWheel()
        {
            // Seteo la posicion
            blw.Position = new Vector3(v_blwRPos.X, v_blwRPos.Y /*+ s_springNExt - s_oscExt*/, v_blwRPos.Z);

            // Seteo el angulo
            blw.rotateX((s_bwAngVelocity * delta_t) / (2.0f * MathHelper.PI)); 
            
            if (blw.Rotation.X > 6.2857f) 
                blw.rotateX(-6.2857f);

            blw.SetRotationY(body.Rotation.Y);
            //blw.SetRotationZ(body.Rotation.Z + nzWeightAng); // TODO: Chequear esto.
        }

        /*
         * Setear la posicion de la Rueda Trasera Derecha
         */
        public void PositionBackRightWheel()
        {
            // Seteo la posicion
            brw.Position = new Vector3(v_brwRPos.X, v_brwRPos.Y /*+ s_springNExt - s_oscExt*/, v_brwRPos.Z);

            // Seteo el angulo
            brw.rotateX((s_bwAngVelocity * delta_t) / (2.0f * MathHelper.PI)); 
            
            if (brw.Rotation.X > 6.2857f) 
                brw.rotateX(-6.2857f);

            brw.SetRotationY(body.Rotation.Y);
            //brw.SetRotationZ(body.Rotation.Z + nzWeightAng); // TODO: Chequear esto.
        }

        /*
         * Setear la posicion del cuerpo del vehiculo
         */
        public void PositionBody()
        {
            // Calculo las coordenadas del vehiculo
            CalcVehicleCoord();

            // set x and z positions
            //body.Position = new Vector3(v_position.X, make.groundHeight, v_position.Z);
            body.Position = new Vector3(v_position.X, body.Position.Y, v_position.Z);

            /*
            // calculate wheel position ground heights (delayed values)
            float flwH = land->getElandHeight(v_flwRPos.x, v_flwRPos.z, 0);
            float frwH = land->getElandHeight(v_frwRPos.x, v_frwRPos.z, 0);
            float blwH = land->getElandHeight(v_blwRPos.x, v_blwRPos.z, 0);
            float brwH = land->getElandHeight(v_brwRPos.x, v_brwRPos.z, 0);

            body.Position.Y = make.groundHeight; // set y position

            // calculate position values
            float fH = (flwH + frwH) / 2.0f;
            float bH = (blwH + brwH) / 2.0f;
            float lH = (flwH + blwH) / 2.0f;
            float rH = (frwH + brwH) / 2.0f;

            // calculate and set vehicle curved orientation angles
            float xAng = asin((bH - fH) / s_dist(fPos, bPos));
            float zAng = asin((rH - lH) / s_dist(rPos, lPos));
            s_bodyCurveX = s_curveValue(xAng, s_bodyCurveX, 1.001f);
            s_bodyCurveZ = s_curveValue(zAng, s_bodyCurveZ, 1.001f);
            */

            //body.rotateX(nxWeightAng);
            //body.rotateZ(nzWeightAng);
        }

        /*
         * Inicializo todas los vectores del vehiculo
         */
        public void ResetVehicle()
        {
            // Limpio los vectores de fisica
            v_tractionForce = new Vector3();
            v_dragForce = new Vector3();
            v_rollResistanceForce = new Vector3();
            v_longitudinalForce = new Vector3();
            v_velocity = new Vector3();
            v_acceleration = new Vector3();
            v_position = new Vector3();
            v_oldPosition = new Vector3();
            v_brakingForce = new Vector3();
            v_turnFriction = new Vector3();
        }

        /*
         * Proceso la entrada de datos del teclado
         */
        public void ControlInput(TgcD3dInput input, float elapsed_time)
        {

            // Seteo el tiempo
            delta_t = elapsed_time;

            /*
             * Pedal de aceleracion. Se incrementa gradualmente de 0% a 100%
             */
            if (input.keyDown(Key.W))
            {
                breaking = false;			// No esta frenando
                freeMoving = false;			// No esta en libre movimiento

                // Incremento el pedal de aceleracion de 0% a 100%
                s_throttleAmount += 0.1f; 
                
                if (s_throttleAmount > 1.0f) 
                    s_throttleAmount = 1.0f;

                s_maxTorque = GetTorqueValue(s_rpm);
                s_engineForce = (s_throttleAmount * s_maxTorque) * make.gearRatio[s_gear] * make.diffRatio * (make.transEff / make.wheelRadius);
            }
            else
            {
                freeMoving = true;			// Se mueve libremente
                s_engineForce = 0.0f;		// No aplica fuerza del motor
                s_throttleAmount = 0.0f;	// No se esta presionando el pedal del acelerador
            }

            /*
             * Pedal de freno. Se incrementa gradualmente de 0% a 100%
             */
            if (input.keyDown(Key.S))
            {
                breaking = true;			// Se esta frenando

                // Incremento el pedal de freno de 0% a 100%
                s_brakeAmount += 0.01f; 
                
                if (s_brakeAmount > 1.0f) 
                    s_brakeAmount = 1.0f;

                if (s_speed <= 0.5f) 
                { 
                    s_brakeForce = 0.0f; 
                    ResetVehicle(); 
                } 
                else 
                { 
                    s_brakeForce = s_brakeAmount * (make.brakeTorque + s_blwMaxTraction + s_brwMaxTraction + s_flwMaxTraction + s_frwMaxTraction); 
                }
            }
            else
            {
                breaking = false;			// No se esta presionando el pedal de freno
                s_brakeForce = 0.0f;		// No hay resistancia de frenado
                s_brakeAmount = 0.0f;		// No se esta presionando el pedal de freno
            }

            /*
             * Volante de giro. Simulacion del volante del vechiculo que controla la
             * direccion de las ruedas. El angulo de giro es incrementado gradualmente
             * en una direccion con limites de aproximadamente 28 grados.
             */

            // Giro hacia la derecha
            if (input.keyDown(Key.D)) 
                wAng += delta_t * 1.0f;

            // 28 grados (0.4888 radianes) en sentido horario
            if (wAng > 0.4888f) 
                wAng = 0.4888f;	

            // Giro hacia la izquierda
            if (input.keyDown(Key.A)) 
                wAng -= delta_t * 1.0f;
            
            // 28 grados(0.4888 radianes) en sentido anti horario
            if (wAng < -0.4888f) 
                wAng = -0.4888f;	

            // Estabilizar el volante de vuelta al origen
            if ((!input.keyDown(Key.D)) && (!input.keyDown(Key.A)))
            {
                if (wAng > 0.0f) 
                {
                    wAng -= delta_t * 1.0f; 
                    if (wAng < 0.0f) wAng = 0.0f; 
                }

                if (wAng < 0.0f) 
                { 
                    wAng += delta_t * 1.0f; 
                    if (wAng > 0.0f) wAng = 0.0f; 
                }
            }

            /*
             * Palanca de cambios. (Caja de 6ta (1 a 6), reversa = 0)
             * Simula el cambio de la caja de cambios entre 1ra y 6ta. 0 esta reservado
             * para la marcha atras.
             */

            // Aumento o disminucion de cambio
            s_ogear = s_gear; // Guardo el ultimo cambio utilizado

            if (input.keyDown(Key.Up)) 
            { 
                if (gKey) 
                { 
                    gKey = false; 
                    s_gear += 1; 

                    if (s_gear > 6) 
                        s_gear = 6; 
                } 
            }

            if (input.keyDown(Key.Down)) 
            { 
                if (gKey) 
                { 
                    gKey = false; 
                    s_gear -= 1; 
                    if (s_gear < 0) 
                        s_gear = 0; 
                } 
            }

            if ((!input.keyDown(Key.Up)) && (!input.keyDown(Key.Down)))
            { 
                gKey = true; 
            }

            if (s_ogear != s_gear) s_throttleAmount = 0.0f;	// Simula que se solto el pedal de aceleracion

        }

        /*
         * Seteo los datos obtenidos.
         */
        public void ControlOutput()
        {
            // Cambio la orientacion del vehiculo segun la velocidad angular
            float s_newAngleY = 0.0f;
            if (s_dir == 1)
            {
                if (wAng > 0.0f) { s_newAngleY = body.Rotation.Y + (s_omega * delta_t); body.SetRotationY(s_newAngleY); }
                if (wAng < 0.0f) { s_newAngleY = body.Rotation.Y + (s_omega * delta_t); body.SetRotationY(s_newAngleY); }
            }
            else
            {
                if (wAng > 0.0f) { s_newAngleY = body.Rotation.Y - (s_omega * delta_t); body.SetRotationY(s_newAngleY); }
                if (wAng < 0.0f) { s_newAngleY = body.Rotation.Y - (s_omega * delta_t); body.SetRotationY(s_newAngleY); }
            }

            PositionBody();				// Posicionar el cuerpo del vehiculo
            TransformWheelPos();		// Transformar las posiciones de las ruedas
            PositionFrontLeftWheel();	// Posicionar la Rueda Frontal Izquierda del vehiculo
            PositionFrontRightWheel();  // Posicionar la Rueda Frontal Derecha del vehiculo
            PositionBackLeftWheel();	// Posicionar la Rueda Trasera Izquierda del vehiculo
            PositionBackRightWheel();	// Posicionar la Rueda Trasera Derecha del vehiculo
            TransformCOG();				// Transformar el Centro de Gravedad
        }

        /*
         * Lookup curve: Devuelve el torque segun las rpm
         * Interpolacion
         */
        public float GetTorqueValue(float rpm)
        {
            // Casos extremos
            if (rpm <= make.engine[0].rpm) return make.engine[0].torque;
            if (rpm >= make.engine[5].rpm) return make.engine[5].torque;

            for (int x = 0; x < 5; x++)
            {
                if (rpm == make.engine[x].rpm)
                {
                    return make.engine[x].torque; // exact rpm value
                }
                else
                {

                    // Interpolacion linear
                    if ((rpm > make.engine[x].rpm) && (rpm < make.engine[x + 1].rpm))
                    {
                        float rpmDiff = make.engine[x + 1].rpm - make.engine[x].rpm;
                        float trqDiff = make.engine[x + 1].torque - make.engine[x].torque;
                        float valDiff = rpm - make.engine[x].rpm;
                        float newDiff = (valDiff / rpmDiff) * trqDiff;

                        return make.engine[x].torque + newDiff;
                    }
                }
            }

            return 0.0f;
        }

        /*
         * Proceso todos los datos. La fisica del motor.
         */
        public void ControlVehicle(TgcD3dInput input, float elapsed_time)
        {	
            // ---                                ---
            // --- Calculos externos del vehiculo ---
            // ---                                ---

	        ControlInput(input, elapsed_time);	// Proceso la entrada de datos del teclado
	        CalcWheelWeight();	                // Calculo la fuerza peso de las ruedas TODO: Chequear esto											
	        CalcWeightForce();	                // Calculo la fuerza peso TODO: Chequear esto
	        // CalcSuspension();	            // TODO: Chequear esto

            // ---                                           ---
            // --- Determino la fuerza de traccion del motor ---
            // ---                                           ---

            // Calculo la maxima traccion en las ruedas (friccion)
	        s_blwMaxTraction = MathHelper.Mu * s_blwWeight;
	        s_brwMaxTraction = MathHelper.Mu * s_brwWeight;
	        s_flwMaxTraction = MathHelper.Mu * s_flwWeight;
	        s_frwMaxTraction = MathHelper.Mu * s_frwWeight;

            // Calculo la friccion en las ruedas traseras (Determina el toruqe maximo aplicado)
	        s_blwMaxTorque = s_blwMaxTraction * make.wheelRadius;
	        s_brwMaxTorque = s_brwMaxTraction * make.wheelRadius;

            // Calculo la friccion por giro
	        float sFact = FastMath.Exp(s_speed / 100.0f) - 1.0f;
	        float tFric = sFact * (FastMath.Abs(wAng) / (MathHelper.PI / 2.0f)) * (s_flwMaxTraction + s_frwMaxTraction);
	        
            if(s_dir == 1) 
                v_turnFriction = -vDir * tFric ;
            else 
                v_turnFriction =  vDir * tFric ;

            // Calculo la fuerza de traccion ( < Traccion maxima por no tener slip)
	        if(s_gear==0) 
                s_absoluteForce =-s_engineForce + s_weightForce;
            else 
                s_absoluteForce = s_engineForce + s_weightForce;

	        v_tractionForce = vDir * s_absoluteForce;

	        if(!freeMoving) 
                s_netTorque = s_absoluteForce;

	        if(breaking) 
                s_netTorque = -s_brakeForce;

            // ---                                                                        ---
            // --- Determino las velocidades angulares de las ruedas delateras y traseras ---
            // ---                                                                        ---

	        s_speed = v_velocity.Length(); // Calculo la velocidad (escalar)

            // Calculo la fuerza de frenado
	        if(s_dir == 1) 
                v_brakingForce =-vDir * s_brakeForce;
            else 
                v_brakingForce = vDir * s_brakeForce;
        

            // Calculo la velocidad angular de las ruedas delanteras (depende de la velocidad escalar)
	        if(s_dir == 1)
		        s_fwAngVelocity = (1.0f - s_brakeAmount) * (s_speed / make.wheelRadius);
	        else 
		        s_fwAngVelocity = -1.0f * (1.0f - s_brakeAmount) * (s_speed / make.wheelRadius);


            // Calculo la velocidad anglar de las ruedas traseras (dependen de la velocidad del motor)
	        s_wheelIntertia = make.wheelMass * (FastMath.Pow2(make.wheelRadius)) / 2.0f;

            // Inercia arbitraria extra inertia = intertia of axle + gears + engine etc
	        float s_totalInertia = (2 * s_wheelIntertia) + 5000.0f; 
	        s_bwAngAcceleration = s_netTorque / s_totalInertia;

	        if(freeMoving)
            {
		        s_bwAngVelocity = s_fwAngVelocity;
	        } 
            else 
            {
		        if(s_dir == 1)
		        {
			        s_bwAngVelocity += s_bwAngAcceleration * delta_t;

			        if(s_bwAngVelocity < s_fwAngVelocity ) 
                        s_bwAngVelocity = s_fwAngVelocity;

                    // Bloqueamos la velocidad si es limite es excedido
			        if(s_rpm >= make.engine[5].rpm) 
                        s_bwAngVelocity = s_fwAngVelocity; 
		        } 
                else 
			        s_bwAngVelocity = s_fwAngVelocity;
	        }


            // ---                                                                                ---
            // --- Calcular la fuerza longitudinar que actua sobre el vehiculo (fuerza principal) ---
            // ---                                                                                ---

            // Calcular las revoluciones por minuto del motor, si las RPM son excedidas
            // y constantemente se mantiene sobre el limite, el motor explota despues de
            // un cierto tiempo

	        if(s_dir==1)
	        {
		        s_rpm =  s_bwAngVelocity * make.gearRatio[s_gear] * make.diffRatio * (60.0f / (2.0f * MathHelper.PI));
	        } 
            else
            {
		        s_rpm = -s_bwAngVelocity * make.gearRatio[s_gear] * make.diffRatio * (60.0f / (2.0f * MathHelper.PI));
	        }

            // RPM limite inferior
	        if(s_rpm < make.engine[0].rpm) 
                s_rpm = make.engine[0].rpm; 

	        v_dragForce = -s_dragConstant * v_velocity * s_speed;           // Calculo la fuerza aerodinamica del vehiculo (Drag)
	        v_rollResistanceForce = -s_rollResistanceConstant * v_velocity;	// Calculo la fuerza de rozamiento (Rolling resistance)

            // Control de aplicacion de fuerzas de frenado/traccion
	        if(breaking) 
	        {
		        v_longitudinalForce = v_brakingForce + v_dragForce + v_rollResistanceForce;
	        } 
            else
            {
		        if(s_rpm <= make.engine[5].rpm)
		        {
			        v_longitudinalForce = v_tractionForce + v_dragForce + v_rollResistanceForce + v_turnFriction; // Debajo del limite de RPM
		        } 
                else {

                    // Ocurre cuando se baja un cambio en una RPM mas alta que la que el cambio inferior puede tomar
                    // entonces el motor baja la velocidad de las ruedas.

			        v_longitudinalForce = v_dragForce + (100.0f * v_rollResistanceForce) + v_turnFriction; // Sobre el limite de RPM
		        }
	        }

            // ---                                                                 ---
            // Determino la posicion del vehiculo basada en la fuerza longitudinal ---
            // ---                                                                 ---

            // Calcular la aceleracion del vehiculo
	        v_acceleration.X = v_longitudinalForce.X / make.mass;
            v_acceleration.Y = v_longitudinalForce.Y / make.mass;
            v_acceleration.Z = v_longitudinalForce.Z / make.mass; 

            // Rotar el vector de velocidad para matchear con la orientacion del cuerpo
	        if(s_dir == 1) 
                v_velocity = vDir * s_speed;
            else 
                v_velocity = vDir * -s_speed;

            // Calcular la velocidad del vehiculo
	        v_velocity = v_velocity + (delta_t * v_acceleration);

            // Calcular el slip ratio (depende de diferentes velocidades de las ruedas traseras)
	        if(s_speed == 0.0f) 
                s_slipRatio = 0.0f;
            else
                s_slipRatio = ((s_bwAngVelocity * make.wheelRadius) - s_speed) / s_speed;

            // Obtener la vieja posicion del cuerpo del vehiculo
	        v_oldPosition = body.Position;

            // Calcular la nueva posicion del cuerpo del vehiculo
	        v_position = v_oldPosition + (delta_t * v_velocity);

            // rem determine movement direction (one time)
	        if(s_speed < 0.1f) 
	        {
		        float ang = MathHelper.Angle(v_position - v_oldPosition, vDir);
		        
                if(ang<1.57) 
                    s_dir = 1;
                else
                    s_dir =-1;
		        
                if(s_speed == 0.0f) s_dir = 0;
	        }

	        // ---                                  ---
            // --- Actualizo el output del vehiculo ---
            // ---                                  ---

	        ControlOutput();

            // ---                         ---
            // --- Calcular valores varios ---
            // ---                         ---

            // Calcular los vectores de direccion del vehiculo
	        CalcDirVect();

            // Calcular el sideslip angle del cuerpo
	        if(bDir != vDir)
            {
                s_beta = FastMath.Acos(Vector3.Dot(bDir, vDir) / (bDir.Length() * vDir.Length()));
                
                if(wAng < 0.0f) 
                    s_beta = -s_beta;
            }
            else
                s_beta = 0.0f;
	
            // Calcular el sideslip angle de las ruedas frontales
	        if(flwDir != fwvDir)
            {
                s_frontAlpha = FastMath.Acos(Vector3.Dot(flwDir, fwvDir) / (flwDir.Length() * fwvDir.Length()));

                if(wAng < 0.0f) 
                    s_frontAlpha = -s_frontAlpha;
            }
            else
                s_frontAlpha = 0.0f;

            // Seteo el sidelslip angle de las ruedas traseras
	        s_rearAlpha  = s_beta; 

            // Calcular el coeficiente de giro (cornering stiffness)
	        s_frontCornerStiff = 0.27f * (s_flwWeight + s_frwWeight);
	        s_rearCornerStiff  = 0.27f * (s_blwWeight + s_brwWeight);

            // Calcular la fuerza centripetal del vehiculo
	        s_frontLateralForce	= 1.5f * s_frontCornerStiff * MathHelper.Deg(s_frontAlpha);
            s_rearLateralForce = 1.5f * s_rearCornerStiff * MathHelper.Deg(s_rearAlpha);
	        s_centripetalForce	= FastMath.Abs(s_rearLateralForce + (FastMath.Cos(wAng) * s_frontLateralForce));

            // Calcular el radio de curvatura a altas velocidades
	        s_hsRadius = (make.mass * FastMath.Pow2(s_speed)) / s_centripetalForce;

	        CalcAngVel();		// Calcular la velocidad angular a baja velocidad
	        CalcAngVel2();		// Calcular la velocidad angular a alta velocidad


            // PrintDebugInfo();
            /*
	        printDebugInfo();	// print debug information
	        controlHUD();		// control the vehicle HUD
	        controlSounds();	// control the vehicle sounds

	        // controls the vehicle camera
	        if(camTyp==0) cam.curveFollow(*body,*land,20.0f,10.0f,5.0f,1.003f,3.0f);	// 3rd person camera
	        if(camTyp==1) cam.orbit(*body,15.0f,7.0f,0.0f,0.001f);						// orbital camera
	        if(camTyp==2) labtec.controlCamera(70.0f,100.0f);							// freelook camera
            */
        }

        public void ResetVariables()
        {
            // Inicializo variables varisa
			gKey						= true;
			freeMoving					= false;
			breaking					= false;
			wAng						= 0.0f;

			// Seteo variables fisicas
			s_dragConstant				= 0.50f * make.drag * make.frontArea * MathHelper.Rho;
			s_rollResistanceConstant	= 30.0f * s_dragConstant;
			wAng						= 0.0f;
			oWang						= 0.0f;
			nxWeightAng					= 0.0f;	
			oxWeightAng					= 0.0f;
			nzWeightAng					= 0.0f;	
			ozWeightAng					= 0.0f;
			s_throttleAmount			= 0.0f;
			s_brakeAmount				= 0.0f;
			s_maxTorque					= 0.0f;
			s_gear						= 1;
			s_ogear						= 1;
			s_dir						= 0;
			s_fwAngVelocity				= 0.0f;
			s_bwAngVelocity				= 0.0f;
			s_bwAngAcceleration			= 0.0f;
			s_wheelIntertia				= 0.0f;
			s_rpm						= 0.0f;
			s_blwMaxTraction			= 0.0f;
			s_brwMaxTraction			= 0.0f;
			s_flwMaxTraction			= 0.0f;
			s_frwMaxTraction			= 0.0f;
			s_engineForce				= 0.0f;
			s_absoluteForce			    = 0.0f;
			s_weightForce				= 0.0f;
			s_flwWeight					= 0.0f;
			s_frwWeight					= 0.0f;				
			s_blwWeight					= 0.0f;				
			s_brwWeight					= 0.0f;	
			s_slipRatio					= 0.0f;
			s_netTorque					= 0.0f;
			s_frontAlpha				= 0.0f;
			s_rearAlpha					= 0.0f;
			s_beta						= 0.0f;
			s_frontCornerStiff			= 0.0f;
			s_rearCornerStiff			= 0.0f;
			s_centripetalForce			= 0.0f;
			s_frontLateralForce			= 0.0f;
			s_rearLateralForce			= 0.0f;
			s_lsRadius					= 0.0f;
			s_hsRadius					= 0.0f;
			s_vehicleWeight				= MathHelper.g * make.mass;
			s_wheelWeight				= MathHelper.g * make.wheelMass;
			s_omega						= 0.0f;
			s_omega2					= 0.0f;
			s_inertia					= 0.0833f * make.mass * (FastMath.Pow2(make.bodyLength) + FastMath.Pow2(make.bodyWidth) + FastMath.Pow2(make.bodyHeight));

			s_bodyCurveX				= 0.0f;
			s_bodyCurveY				= 0.0f;
			s_bodyCurveZ				= 0.0f;
			s_flwCurveAng				= 0.0f;
			s_frwCurveAng				= 0.0f;

			// clear vector memories
			fwvDir = new Vector3();
			flwDir = new Vector3();
			frwDir = new Vector3();
			flwNDir = new Vector3();
			frwNDir = new Vector3();
			blwNDir = new Vector3();
			brwNDir = new Vector3();
		}

        public void Render()
        {
            // Render the body and tyres
            body.render();
            flw.render();
            frw.render();
            blw.render();
            brw.render();
        }

        public Vector3 GetPosition()
        {
            Matrix m = body.Transform;
            return new Vector3(m.M41, m.M42, m.M43);
        }

        public Vector3 XAxis()
        {
            Matrix m = body.Transform;
            return new Vector3(m.M11, m.M12, m.M13);
        }

        public Vector3 YAxis()
        {
            Matrix m = body.Transform;
            return new Vector3(m.M21, m.M22, m.M23);
        }

        public Vector3 ZAxis()
        {
            Matrix m = body.Transform;
            return new Vector3(m.M31, m.M32, m.M33);
        }

        public void PrintDebugInfo()
        {
            sw.WriteLine("--------------------------");
            sw.WriteLine("now traction: " + v_tractionForce.Length());
            sw.WriteLine("acceleration: " + v_acceleration.Length());
            sw.WriteLine("time factor: " + delta_t);
            sw.WriteLine("flw weight: " + s_flwWeight);
            sw.WriteLine("frw weight: " + s_frwWeight);
            sw.WriteLine("blw weight: " + s_blwWeight);
            sw.WriteLine("brw weight: " + s_brwWeight);
            sw.WriteLine("max torque: " + s_maxTorque);
            sw.WriteLine("longitudinal: " + v_longitudinalForce.Length());
            sw.WriteLine("v_flwRPos: " + v_flwRPos);
            sw.WriteLine("v_frwRPos: " + v_frwRPos);
            sw.WriteLine("v_blwRPos: " + v_blwRPos);
            sw.WriteLine("v_brwRPos: " + v_brwRPos);
        }

    }
}
