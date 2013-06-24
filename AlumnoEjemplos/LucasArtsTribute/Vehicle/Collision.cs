using Microsoft.DirectX;
using TgcViewer.Utils.TgcGeometry;

namespace AlumnoEjemplos.LucasArtsTribute
{
    public class Collision
    {
        private const float Epsilon2 = 0.01f;

        /*public static bool TestOBB_Vs_OBB(OrientedBoundingBox a, OrientedBoundingBox b)
        {
            LineSegment2D[] segA = GetSegmentInXZ(GetVertexInXZ(a));
            LineSegment2D[] segB = GetSegmentInXZ(GetVertexInXZ(b));


            for (int j = 0; j < 4; j++)
            {
                for (int i = 0; i < 4; i++)
                {
                    if (segA[i].IntersectWithSegment(segB[j]))
                        return true;
                }
            }
            return false;
        }*/

        public static bool TestOBB_Vs_OBB(OrientedBoundingBox a, OrientedBoundingBox b)
        {
            LineSegment2D[] segA = GetSegmentInXZ(GetVertexInXZ(a));
            LineSegment2D[] segB = GetSegmentInXZ(GetVertexInXZ(b));


            for (int j = 0; j < 4; j++)
            {
                for (int i = 0; i < 4; i++)
                {
                    if (segA[i].IntersectWithSegment(segB[j]))
                        return true;
                }
            }
            return false;
        }

        public static LineSegment2D[] GetSegmentInXZ(Vector2[] vert)
        {
            LineSegment2D[] seg = new LineSegment2D[4];

            seg[0] = new LineSegment2D(vert[0], vert[1]);
            seg[1] = new LineSegment2D(vert[1], vert[2]);
            seg[2] = new LineSegment2D(vert[2], vert[3]);
            seg[3] = new LineSegment2D(vert[3], vert[0]);

            return seg;
        }

        public static Vector2[] GetVertexInXZ(OrientedBoundingBox a)
        {
            Vector3[] vertex3D = a.computeCorners();

            Vector2[] vertex2D = new Vector2[4];

            vertex2D[0] = new Vector2(vertex3D[1].X, vertex3D[1].Z);
            vertex2D[1] = new Vector2(vertex3D[2].X, vertex3D[2].Z);
            vertex2D[2] = new Vector2(vertex3D[6].X, vertex3D[6].Z);
            vertex2D[3] = new Vector2(vertex3D[5].X, vertex3D[5].Z);

            return vertex2D;
        }

    }


    public class LineSegment2D
    {
        Vector2 pointA;
        Vector2 pointB;

        public LineSegment2D(Vector2 p1, Vector2 p2)
        {
            pointA.X = (int)p1.X;
            pointA.Y = (int)p1.Y;
            pointB.X = (int)p2.X;
            pointB.Y = (int)p2.Y;
        }

        public bool IntersectWithSegment(LineSegment2D segmento)
        {
            return AreSegmentIntersecting(pointA, pointB, segmento.pointA, segmento.pointB);
        }


        public static bool IsCounterClockWise(Vector2 punto1, Vector2 punto2, Vector2 punto3)
        {
            return (punto3.Y - punto1.Y) * (punto2.X - punto1.X) > (punto2.Y - punto1.Y) * (punto3.X - punto1.X);
        }


        //a y b definen los puntos del primer segmento de recta, c y d el del segundo segmento de recta
        public static bool AreSegmentIntersecting(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
        {
            if (IsCounterClockWise(a, c, d) == IsCounterClockWise(b, c, d))
                return false;
            if (IsCounterClockWise(a, b, c) == IsCounterClockWise(a, b, d))
                return false;
            return true;
        }

    }
}