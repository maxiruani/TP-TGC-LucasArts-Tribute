using Microsoft.DirectX;
using TgcViewer.Utils.TgcGeometry;

namespace AlumnoEjemplos.LucasArtsTribute.Models
{
    public class Vector
    {
        public float X;
        public float Y;
        public float Z;

        public Vector()
        {
        }

        public Vector(float X, float Y, float Z)
        {
            this.X = X;
            this.Y = Y;
            this.Z = Z;
        }

        // Premultiplicacion por matriz
        public static Vector operator *(Vector v, Matrix m)
        {
            Vector[] u = new Vector[3];
            u[0] = new Vector(m.M11, m.M12, m.M13);
            u[1] = new Vector(m.M21, m.M22, m.M23);
            u[2] = new Vector(m.M31, m.M32, m.M33);

            return new Vector(v * u[0], v * u[1], v * u[2]);
        }

        // Inverso aditivo de un vector
        public static Vector operator -(Vector v)
        {
            return new Vector(-v.X, -v.Y, -v.Z);
        }

        // Suma de vectores
        public static Vector operator +(Vector v1, Vector v2)
        {
            return new Vector(v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z);
        }

        // Resta de vectores
        public static Vector operator -(Vector v1, Vector v2)
        {
            return new Vector(v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z);
        }

        // Multiplicacion de un escalar por un vector
        public static Vector operator *(float e, Vector v1)
        {
            return new Vector(v1.X * e, v1.Y * e, v1.Z * e);
        }

        // Multiplicacion de un vector por un escalar
        public static Vector operator *(Vector v1, float e)
        {
            return new Vector(v1.X * e, v1.Y * e, v1.Z * e);
        }

        // Division de un vector por un escalar
        public static Vector operator /(Vector v1, float e)
        {
            return new Vector(v1.X / e, v1.Y / e, v1.Z / e);
        }

        // Producto escalar entre vectores
        public static float operator *(Vector v1, Vector v2)
        {
            return v1.X * v2.X + v1.Y * v2.Y + v1.Z * v2.Z;
        }

        // Modulo del vector |vector|
        public float Length()
        {
            return FastMath.Sqrt(this * this);
        }

        // Transformamos vector en coordenadas con un angulo a respecto del eje Y a coordenadas del mundo
        public Vector FromAngleToY(float a)
        {
            float sn = FastMath.Sin(a);
            float cs = FastMath.Cos(a);

            return new Vector(new Vector(sn, 0, cs) * this, 0, new Vector(cs, 0, -sn) * this);
        }

        // Devuelve un Vector3 de DirectX
        public Vector3 ToDirectXVector()
        {
            return new Vector3(this.X, this.Y, this.Z);
        }

        // Devuelve un vector igual a si mismo
        public Vector Clone()
        {
            return new Vector(this.X, this.Y, this.Z);
        }

        // Devuelve su versor
        public Vector ToVersor()
        {
            if (this.X == 0 && this.Y == 0 && this.Z == 0)
            {
                return this;
            }
            else
            {
                return this / this.Length();
            }
        }

        public override string ToString()
        {
            return X + " " + Y + " " + Z;
        }

    }
}
