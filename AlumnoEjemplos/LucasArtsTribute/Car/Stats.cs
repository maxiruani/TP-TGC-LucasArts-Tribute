using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using AlumnoEjemplos.LucasArtsTribute.Models;

namespace AlumnoEjemplos.LucasArtsTribute
{
    public class CarStats
    {
        public float Zmin { get; set; }
        public float Zmax { get; set; }

        public float Velocity { get; set; }
        public float Acceleration { get; set; }
        public float Gear { get; set; }
        /*
        public float Velocity
        {
            get { return velocity_wc.Length(); }
        }

        public float Acceleration
        {
            get { return acceleration_wc.Length(); }
        }

        public Vector velocity_wc { get; set; }
        public Vector acceleration_wc { get; set; }
        */
    }


}
