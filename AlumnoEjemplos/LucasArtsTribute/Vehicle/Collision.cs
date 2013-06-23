using Microsoft.DirectX;
using TgcViewer.Utils.TgcGeometry;

namespace AlumnoEjemplos.LucasArtsTribute
{
    public class Collision
    {
        private const float Epsilon2 = 0.01f;

        public static bool TestOBB_Vs_OBB(OrientedBoundingBox a, OrientedBoundingBox b)
        {
            LineSegment2D[] segA = dameSegmentosEnXZ(dameVerticesEnXZ(a));
            LineSegment2D[] segB = dameSegmentosEnXZ(dameVerticesEnXZ(b));


            for (int j = 0; j < 4; j++)
            {
                for (int i = 0; i < 4; i++)
                {
                    if (segA[i].intersectaSegmento(segB[j]))
                        return true;
                }
            }
            return false;
        }

        public static LineSegment2D[] dameSegmentosEnXZ(Vector2[] vert)
        {
            LineSegment2D[] seg = new LineSegment2D[4];

            seg[0] = new LineSegment2D(vert[0], vert[1]);
            seg[1] = new LineSegment2D(vert[1], vert[2]);
            seg[2] = new LineSegment2D(vert[2], vert[3]);
            seg[3] = new LineSegment2D(vert[3], vert[0]);

            return seg;
        }

        public static Vector2[] dameVerticesEnXZ(OrientedBoundingBox a)
        {
            Vector3[] vertices3D = a.ComputeCorners();

            Vector2[] vertices2D = new Vector2[4];

            vertices2D[0] = new Vector2(vertices3D[1].X, vertices3D[1].Z);
            vertices2D[1] = new Vector2(vertices3D[2].X, vertices3D[2].Z);
            vertices2D[2] = new Vector2(vertices3D[6].X, vertices3D[6].Z);
            vertices2D[3] = new Vector2(vertices3D[5].X, vertices3D[5].Z);

            return vertices2D;
        }

    }


    public class LineSegment2D
    {
        Vector2 punto1;
        Vector2 punto2;


        public LineSegment2D(Vector2 p1, Vector2 p2)
        {
            punto1 = p1;
            punto2 = p2;
        }




        public bool intersectaSegmento(LineSegment2D segmento)
        {
            return intersectanSegmentos(punto1, punto2, segmento.punto1, segmento.punto2);
        }


        public static bool isCounterClockWise(Vector2 punto1, Vector2 punto2, Vector2 punto3)
        {
            return (punto3.Y - punto1.Y) * (punto2.X - punto1.X) > (punto2.Y - punto1.Y) * (punto3.X - punto1.X);
        }


        //a y b definen los puntos del primer segmento de recta, c y d el del segundo segmento de recta
        public static bool intersectanSegmentos(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
        {
            if (isCounterClockWise(a, c, d) == isCounterClockWise(b, c, d))
                return false;
            else if (isCounterClockWise(a, b, c) == isCounterClockWise(a, b, d))
                return false;
            else
                return true;
        }

    }
}