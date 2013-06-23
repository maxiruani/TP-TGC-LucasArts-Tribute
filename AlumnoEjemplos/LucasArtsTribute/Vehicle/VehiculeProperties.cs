using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;

using AlumnoEjemplos.LucasArtsTribute.Utils;

namespace AlumnoEjemplos.LucasArtsTribute.VehicleModel
{
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

        public string engineSnd;			// path of the engine sound
        public string hornSnd;				// path of the horn sound
        public string brakeSnd;				// path of the brake sound

        public float mass;					// vehicle mass
        public float wheelMass;				// vehicle wheel mass
        public float drag;					// coefficient of frition
        public float frontArea;				// frontal area
        public float bodyLength;			// length of vehicle body
        public float bodyWidth;				// width of vehicle body
        public float bodyHeight;			// height of vehicle body
        public float wheelRadius;			// radius of the vehicle wheel


        public float groundHeight;			// height between vehicle and ground

        public float brakeTorque;			// torque applied to lock wheels
        public float[] gearRatio;			// stores the gear 1 to 6 ratios (0 is reverse)
        public float diffRatio;				// stores the differential ratio
        public float transEff;			    // stores the transmission efficiency

        public Vector3 cog;				    // vehicle centre of gravity (z axis offset)
        public Vector3 fl;					// default position of front left wheel
        public Vector3 fr;					// default position of front right wheel
        public Vector3 bl;					// default position of back left wheel
        public Vector3 br;					// default position of back right wheel
        public Vector3 bScl;				// scale factors for the vehicle body
        public Vector3 wScl;				// scale factors for the vehicle tyre

        public EngineProperties[] engine;	// rpm/torque lookup curve container

        private VehicleProperties()
        {
        }

        public static VehicleProperties Create(String path)
        {
            ConfigurationManager config = new ConfigurationManager();

            VehicleProperties make = new VehicleProperties();

            try
            {
                if (!config.ReadConfigFile(path))
                    return null;

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

                make.engineSnd = config.GetValue("engineSnd");
                make.hornSnd = config.GetValue("hornSnd");
                make.brakeSnd = config.GetValue("brakeSnd");
            }
            catch (Exception e)
            {
                return null;
            }

            return make;
        }

    }
}
