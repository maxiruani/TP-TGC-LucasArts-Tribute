using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;
using TgcViewer;
using TgcViewer.Utils.TgcGeometry;

namespace AlumnoEjemplos.LucasArtsTribute.Utils
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
        public static float WrapValue(float a)
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
}
