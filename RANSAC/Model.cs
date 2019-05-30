using System;
using System.Collections.Generic;
using System.Text;

namespace RANSAC
{
    public class Model
    {
        private Plane plane;
        private int[] inliers;
        private Vector3[] points;

        public Plane Plane
        {
            get
            {
                return plane;
            }
        }

        public float A
        {
            get
            {
                return plane.A;
            }
        }

        public float B
        {
            get
            {
                return plane.B;
            }
        }

        public float C
        {
            get
            {
                return plane.C;
            }
        }

        public float D
        {
            get
            {
                return plane.Offset;
            }
        }

        public Vector3[] Inliers
        {
            get
            {
                List<Vector3> result = new List<Vector3>();
                foreach(int x in inliers)
                {
                    result.Add(points[x]);
                }

                return result.ToArray();
            }
        }

        public Model(Plane plane, int[] inliers, Vector3[] points)
        {
            this.plane = plane;
            this.inliers = new int[inliers.Length];
            this.points = new Vector3[points.Length];
            Array.Copy(inliers, this.inliers, inliers.Length);
            Array.Copy(points, this.points, points.Length);
            //this.inliers = inliers;
            //this.points = points;
        }

        public override string ToString()
        {
            return ToString("F6", System.Globalization.CultureInfo.InvariantCulture);
        }

        public string ToString(string specifier, IFormatProvider formatProvider)
        {
            formatter6 f = new formatter6();
            f.specifier = specifier;
            f.provider = formatProvider;            

            return String.Format("{0}\t{1}\t{2}\t{3}",
                f.s(A), f.s(B), f.s(C), f.s(D));
        }

        private class formatter6
        {
            public IFormatProvider provider;
            public string specifier = "F6";

            public string s(double x)
            {
                if (provider == null)
                    provider = System.Globalization.CultureInfo.CurrentCulture;

                string str = x.ToString(specifier, provider);
                return str;
            }
        }
    }
}
