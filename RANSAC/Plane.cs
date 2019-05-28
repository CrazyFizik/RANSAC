using System;
using System.Collections.Generic;
using System.Text;

namespace RANSAC
{
    [Serializable]
    public class Plane : IEquatable<Plane>, IFormattable
    {
        [NonSerialized]
        Vector3 normal; 
        float offset;

        public Plane(float a, float b, float c)
        {
            normal = new Vector3(a, b, c);
        }

        public Plane(Vector3 normal)
        {
            this.normal = normal;
        }

        public Plane(float a, float b, float c, float offset)
        {
            this.normal = new Vector3(a, b, c);
            this.offset = offset;
        }

        public Plane(Vector3 normal, float offset)
        {
            this.normal = normal;
            this.offset = offset;
        }
        
        public static Plane FromPoints(Vector3 point1, Vector3 point2, Vector3 point3)
        {
            var x2m1 = point2.X - point1.X;
            var y2m1 = point2.Y - point1.Y;
            var z2m1 = point2.Z - point1.Z;

            var x3m1 = point3.X - point1.X;
            var y3m1 = point3.Y - point1.Y;
            var z3m1 = point3.Z - point1.Z;

            var a = (y2m1 * z3m1) - (z2m1 * y3m1);
            var b = (z2m1 * x3m1) - (x2m1 * z3m1);
            var c = (x2m1 * y3m1) - (y2m1 * x3m1);

            var d = -(a * point1.X + b * point1.Y + c * point1.Z);

            return new Plane(a, b, c, d);
        }

        public static Plane GetPlaneFromPoints(Vector3 point1, Vector3 point2, Vector3 point3)
        {
            var a = point1.Y * (point2.Z - point3.Z) + point2.Y * (point3.Z - point1.Z) + point3.Y * (point1.Z - point2.Z);
            var b = point1.Z * (point2.X - point3.X) + point2.Z * (point3.X - point1.X) + point3.Z * (point1.X - point2.X);
            var c = point1.X * (point2.Y - point3.Y) + point2.X * (point3.Y - point1.Y) + point3.X * (point1.Y - point2.Y);
            var d = -(a * point1.X + b * point1.Y + c * point1.Z);

            return new Plane(a, b, c, d);
        }

        public Vector3 Normal
        {
            get { return normal; }
        }

        /// <summary>
        ///   Gets or sets the constant <c>a</c> in the plane
        ///   definition <c>a * x + b * y + c * z + d = 0</c>.
        /// </summary>
        /// 
        public float A
        {
            get { return normal.X; }
            set { normal.X = value; }
        }

        /// <summary>
        ///   Gets or sets the constant <c>b</c> in the plane
        ///   definition <c>a * x + b * y + c * z + d = 0</c>.
        /// </summary>
        /// 
        public float B
        {
            get { return normal.Y; }
            set { normal.Y = value; }
        }

        /// <summary>
        ///   Gets or sets the constant <c>c</c> in the plane
        ///   definition <c>a * x + b * y + c * z + d = 0</c>.
        /// </summary>
        /// 
        public float C
        {
            get { return normal.Z; }
            set { normal.Z = value; }
        }

        /// <summary>
        ///   Gets or sets the distance offset 
        ///   between the plane and the origin.
        /// </summary>
        /// 
        public float Offset
        {
            get { return offset; }
            set { offset = value; }
        }


        public double DistanceToPoint(Vector3 point)
        {
            var a = normal.X;
            var b = normal.Y;
            var c = normal.Z;

            var num = Math.Abs(a * point.X + b * point.Y + c * point.Z + offset);
            var den = Math.Sqrt(a * a + b * b + c * c);

            return num / den;
        }

        /// <summary>
        ///   Normalizes this plane by dividing its components
        ///   by the <see cref="Normal"/> vector's norm.
        /// </summary>
        /// 
        public void Normalize()
        {
            var norm = normal.Length();
            normal = Vector3.Divide(normal, norm);
            offset /= norm;
        }

        public static bool operator ==(Plane a, Plane b)
        {
            if ((object)a == null && (object)b == null)
                return true;
            if ((object)a == null || (object)b == null)
                return false;

            return a.offset == b.offset && a.normal == b.normal;
        }


        public static bool operator !=(Plane a, Plane b)
        {
            if ((object)a == null && (object)b == null)
                return false;
            if ((object)a == null || (object)b == null)
                return true;

            return a.offset != b.offset || a.normal != b.normal;
        }

        public bool Equals(Plane other, double tolerance)
        {
            return (Math.Abs(offset - other.offset) < tolerance)
                && (Math.Abs(normal.X - other.normal.X) < tolerance)
                && (Math.Abs(normal.Y - other.normal.Y) < tolerance)
                && (Math.Abs(normal.Z - other.normal.Z) < tolerance);
        }

        public bool Equals(Plane other)
        {
            return offset == other.offset && normal == other.normal;
        }

        public override bool Equals(object obj)
        {
            Plane other = obj as Plane;
            if (other == null)
                return false;

            return Equals(other);
        }

        public override int GetHashCode()
        {
            return offset.GetHashCode() + 13 * normal.GetHashCode();
        }


        public override string ToString()
        {
            return ToString("g", System.Globalization.CultureInfo.CurrentCulture);
        }

        public string ToString(string specifier, IFormatProvider formatProvider)
        {
            formatter6 f = new formatter6();
            f.specifier = specifier;
            f.provider = formatProvider;

            return String.Format("{0}x {1}y {2}z {3} = 0",
                f.s(A), f.s(B), f.s(C), f.s(Offset));
        }

        private class formatter6
        {
            public IFormatProvider provider;
            public string specifier = "g";

            public string s(double x)
            {
                if (provider == null)
                    provider = System.Globalization.CultureInfo.CurrentCulture;

                x = Math.Truncate(x * 1e+6) / 1e+6;
                string str = x.ToString(specifier, provider);
                return (x > 0) ? "+" + str : str;
            }
        }

    }
}
