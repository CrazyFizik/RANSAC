namespace RANSAC.Test
{
    using System.Collections.Generic;
    using System.Globalization;
    using System.Runtime.CompilerServices;
    using System.Text;
    using System.IO;
    using System;

    namespace RANSAC
    {
        /// <summary>
        /// Only for simply remote online compiling process
        /// Delete later
        /// </summary>
        public static class RANSAC
        {
            public const string INPUT_FILE = "input.txt";
            public const string OUTPUT_FILE = "output.txt";

            public static void Main(string[] args)
            {
                Test();
            }

            public static void Test()
            {
                var dir = Directory.GetCurrentDirectory();
                var rp = ReadPlaneFromFile(dir, INPUT_FILE);
                System.Console.WriteLine(rp.BestModel.ToString());
                SavePlaneToFile(rp, dir, OUTPUT_FILE);
            }

            public static RansacPlane ReadPlaneFromFile(string dir, string filename)
            {
                // Parse
                string fullPathToFile = Path.Combine(dir, filename);
                StreamReader reader = new StreamReader(fullPathToFile);
                //CultureInfo ci = (CultureInfo)CultureInfo.CurrentCulture.Clone();
                //ci.NumberFormat.CurrencyDecimalSeparator = ".";
                string line;
                int counter = 0;
                float probability = 0;
                List<Vector3> points = new List<Vector3>();
                while ((line = reader.ReadLine()) != null)
                {
                    counter++;

                    // Probability row
                    if (counter == 1)
                    {
                        probability = float.Parse(line, CultureInfo.InvariantCulture);
                        continue;
                    }

                    // Number of elemnts row
                    // not use
                    if (counter == 2)
                    {
                        continue;
                    }

                    char[] delimiter = new char[] { '\t' };
                    string[] row = line.Split(delimiter);
                    Vector3 point = new Vector3();
                    point.X = float.Parse(row[0], CultureInfo.InvariantCulture);
                    point.Y = float.Parse(row[1], CultureInfo.InvariantCulture);
                    point.Z = float.Parse(row[2], CultureInfo.InvariantCulture);
                    points.Add(point);
                }

                // Solve
                RansacPlane rp = new RansacPlane();
                rp.Threshold = probability;
                rp.Estimate(points);

                return rp;
            }

            public static void SavePlaneToFile(RansacPlane rp, string dir, string filename)
            {
                string fullPathToFile = Path.Combine(dir, filename);
                StreamWriter writer = new StreamWriter(fullPathToFile);
                writer.Write(rp.BestModel.ToString());
                writer.Close();
            }
        }

        [Serializable]
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
                    foreach (int x in inliers)
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
                    return (x > 0) ? "+" + str : str;
                }
            }
        }

        /// <summary>
        /// Simple RANdom SAmple Consensus (RANSAC) algorithm for plane fitting
        /// See: https://en.wikipedia.org/wiki/Random_sample_consensus
        /// </summary>
        public class RansacPlane
        {
            private double probability = 0.01;
            private double ratio = 0.5;
            private int samples = 3;
            private int size = 3;
            private int maxSampling = 100;
            private int maxEvaluations = 1000;
            private int trialsPerformed = 0;
            private int trialsNeeded = 1000;

            private int[] inliers;
            private Vector3[] points;

            private Plane bestPlane = new Plane(Vector3.One);
            private int[] bestInliners;
            private Model bestModel;

            public double Threshold
            {
                get
                {
                    return probability;
                }
                set
                {
                    probability = value;
                }
            }

            public double InlinersRatio
            {
                get
                {
                    return ratio;
                }
                set
                {
                    ratio = value;
                }
            }

            public int MaximumNumberOfIterations
            {
                get
                {
                    return maxEvaluations;
                }
                set
                {
                    maxEvaluations = value;
                }
            }

            public int TrialsPerformed
            {
                get
                {
                    return trialsPerformed;
                }
            }

            public int TrialsNeeded
            {
                get
                {
                    return trialsNeeded;
                }
            }

            public Model BestModel
            {
                get
                {
                    return bestModel;
                }
            }

            public Plane Estimate(List<Vector3> points)
            {
                return Estimate(points.ToArray());
            }

            public Plane Estimate(Vector3[] points)
            {
                // Initial argument's number checks
                if (points.Length < 3)
                {
                    throw new ArgumentException("At least three points are required to fit a plane");
                }

                this.points = points;
                this.size = this.points.Length;
                this.samples = GetMinimalNumberOfPoints();

                int[] array = new int[size];
                for (int i = 0; i < size; i++)
                {
                    array[i] = i;
                }

                Plane plane = null;
                int[] sample = new int[3];
                int samplings = 0;
                this.trialsPerformed = 0;
                this.trialsNeeded = maxEvaluations;
                while (this.trialsPerformed < this.trialsNeeded && this.trialsPerformed < this.maxEvaluations)
                {
                    samplings = 0;
                    // While the number of samples attempted is less than the maximum limit of attempts
                    for (; samplings < this.maxSampling;)
                    {
                        sample = GetSamples(array);
                        samplings += sample.Length;

                        // If the sampled points are not in a degenerate configuration,
                        if (!IsDegenrate(sample))
                        {
                            // Define plane using the sample with random selection of 3 points
                            plane = DefinePlane(sample);
                            break;
                        }
                    }

                    if (plane == null)
                    {
                        throw new ArgumentException("A model could not be inferred from the data points");
                    }

                    // Evaluate the distances between total points and the model returning the
                    // indices of the points that are inliers (according to indexes).
                    this.inliers = GetInliners(plane);
                    if (this.inliers.Length >= GetMinimalNumberOfPoints())
                    {
                        bestPlane = plane;
                        bestInliners = this.inliers;
                        bestModel = new Model(bestPlane, bestInliners, points);

                        // Update estimate of N, the number of trials to ensure we pick, 
                        // with probability p, a data set with no outliers.
                        // See https://en.wikipedia.org/wiki/Random_sample_consensus#Parameters 
                        double pInlier = (double)inliers.Length / (double)size;
                        double pNoOutliers = 1.0 - System.Math.Pow(pInlier, samples);

                        double num = System.Math.Log(1.0 - probability);
                        double den = System.Math.Log(pNoOutliers);
                        if (den == 0)
                        {
                            trialsNeeded = num == 0 ? 0 : maxEvaluations;
                        }
                        else
                        {
                            trialsNeeded = (int)(num / den);
                        }
                    }

                    trialsPerformed++;
                }

                return bestPlane;
            }

            private Plane DefinePlane(int[] x, bool normalize = true)
            {
                var p1 = points[x[0]];
                var p2 = points[x[1]];
                var p3 = points[x[2]];
                var result = Plane.FromPoints(p1, p2, p3);
                if (normalize)
                    result.Normalize();
                return result;
            }

            private int[] GetInliners(Plane plane)
            {
                var result = new List<int>();
                for (int j = 0; j < size; j++)
                {
                    var point = points[j];
                    var distance2 = plane.DistanceToPoint(point);
                    if (distance2 <= probability)
                    {
                        result.Add(j);
                    }
                }
                return result.ToArray();
            }

            private bool IsDegenrate(int[] indices)
            {
                var p1 = points[indices[0]];
                var p2 = points[indices[1]];
                var p3 = points[indices[2]];

                return Vector3.Collinear(p1, p2, p3);
            }

            private int GetMinimalNumberOfPoints()
            {
                var result = size;
                //result = (int)Math.Ceiling(ratio * size);
                result = (int)Math.Round(ratio * size);
                if (result > size)
                {
                    result = size;
                }
                return result;
            }

            private int[] GetSamples(int[] array)
            {
                Shuffle(array);
                int[] result = new int[3];
                result[0] = array[0];
                result[1] = array[1];
                result[2] = array[2];
                Array.Sort(result);
                Array.Reverse(result);
                return result;
            }

            /// <summary>
            ///  Knuth / Fisher–Yates shuffle
            /// </summary>        
            private void Shuffle(int[] array)
            {
                Random random = new Random();
                int n = array.Length;
                while (n > 1)
                {
                    n--;
                    int i = random.Next(n + 1);
                    int temp = array[i];
                    array[i] = array[n];
                    array[n] = temp;
                }
            }

            public static Vector3[] GetTestA1()
            {
                Vector3[] result = new Vector3[3];
                result[0] = new Vector3(20, 0, 0);
                result[1] = new Vector3(10, -10, 0);
                result[2] = new Vector3(10, 10, 0);

                return result;
            }

            public static Vector3[] GetTestA2()
            {
                Vector3[] result = new Vector3[3];
                result[0] = new Vector3(20, 0, 3);
                result[1] = new Vector3(10, -10, 2);
                result[2] = new Vector3(10, 10, 2);

                return result;
            }

            public static Vector3[] GetTestA3()
            {
                Vector3[] result = new Vector3[10];
                result[0] = new Vector3(20, -10, 0.2f);
                result[1] = new Vector3(20, 0, 0.2f);
                result[2] = new Vector3(20, 10, 0.2f);
                result[3] = new Vector3(15, -10, 0.15f);
                result[4] = new Vector3(15, 0, 0.15f);
                result[5] = new Vector3(15, 10, 0.15f);
                result[6] = new Vector3(10, -10, 0.1f);
                result[7] = new Vector3(10, 10, 0.1f);
                result[8] = new Vector3(20, 18, 1.7f);
                result[9] = new Vector3(15, -15, 1.2f);

                return result;
            }
        }

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


        [Serializable]
        public partial struct Vector3 : IEquatable<Vector3>, IFormattable
        {
            /// <summary>
            /// The X component of the vector.
            /// </summary>
            public Single X;
            /// <summary>
            /// The Y component of the vector.
            /// </summary>
            public Single Y;
            /// <summary>
            /// The Z component of the vector.
            /// </summary>
            public Single Z;

            #region Constructors
            /// <summary>
            /// Constructs a vector whose elements are all the single specified value.
            /// </summary>
            /// <param name="value">The element to fill the vector with.</param>
            public Vector3(Single value) : this(value, value, value) { }


            /// <summary>
            /// Constructs a vector with the given individual elements.
            /// </summary>
            /// <param name="x">The X component.</param>
            /// <param name="y">The Y component.</param>
            /// <param name="z">The Z component.</param>
            public Vector3(Single x, Single y, Single z)
            {
                X = x;
                Y = y;
                Z = z;
            }
            #endregion Constructors

            #region Public Instance Methods
            /// <summary>
            /// Copies the contents of the vector into the given array.
            /// </summary>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void CopyTo(Single[] array)
            {
                CopyTo(array, 0);
            }

            /// <summary>
            /// Copies the contents of the vector into the given array, starting from index.
            /// </summary>
            /// <exception cref="ArgumentNullException">If array is null.</exception>
            /// <exception cref="RankException">If array is multidimensional.</exception>
            /// <exception cref="ArgumentOutOfRangeException">If index is greater than end of the array or index is less than zero.</exception>
            /// <exception cref="ArgumentException">If number of elements in source vector is greater than those available in destination array.</exception>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void CopyTo(Single[] array, int index)
            {
                if (array == null)
                {
                    // Match the JIT's exception type here. For perf, a NullReference is thrown instead of an ArgumentNull.
                    throw new NullReferenceException("Arg_NullArgumentNullRef");
                }
                if (index < 0 || index >= array.Length)
                {
                    throw new ArgumentOutOfRangeException("Arg_ArgumentOutOfRangeException" + index.ToString());
                }
                if ((array.Length - index) < 3)
                {
                    throw new ArgumentException("Arg_ElementsInSourceIsGreaterThanDestination" + index.ToString());
                }
                array[index] = X;
                array[index + 1] = Y;
                array[index + 2] = Z;
            }

            /// <summary>
            /// Returns a boolean indicating whether the given Vector3 is equal to this Vector3 instance.
            /// </summary>
            /// <param name="other">The Vector3 to compare this instance to.</param>
            /// <returns>True if the other Vector3 is equal to this instance; False otherwise.</returns>
            public bool Equals(Vector3 other)
            {
                return X == other.X &&
                       Y == other.Y &&
                       Z == other.Z;
            }
            #endregion Public Instance Methods

            #region Public Static Methods
            /// <summary>
            /// Returns the dot product of two vectors.
            /// </summary>
            /// <param name="vector1">The first vector.</param>
            /// <param name="vector2">The second vector.</param>
            /// <returns>The dot product.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static float Dot(Vector3 vector1, Vector3 vector2)
            {
                return vector1.X * vector2.X +
                       vector1.Y * vector2.Y +
                       vector1.Z * vector2.Z;
            }

            /// <summary>
            /// Returns a vector whose elements are the minimum of each of the pairs of elements in the two source vectors.
            /// </summary>
            /// <param name="value1">The first source vector.</param>
            /// <param name="value2">The second source vector.</param>
            /// <returns>The minimized vector.</returns>
            public static Vector3 Min(Vector3 value1, Vector3 value2)
            {
                return new Vector3(
                    (value1.X < value2.X) ? value1.X : value2.X,
                    (value1.Y < value2.Y) ? value1.Y : value2.Y,
                    (value1.Z < value2.Z) ? value1.Z : value2.Z);
            }

            /// <summary>
            /// Returns a vector whose elements are the maximum of each of the pairs of elements in the two source vectors.
            /// </summary>
            /// <param name="value1">The first source vector.</param>
            /// <param name="value2">The second source vector.</param>
            /// <returns>The maximized vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 Max(Vector3 value1, Vector3 value2)
            {
                return new Vector3(
                    (value1.X > value2.X) ? value1.X : value2.X,
                    (value1.Y > value2.Y) ? value1.Y : value2.Y,
                    (value1.Z > value2.Z) ? value1.Z : value2.Z);
            }

            /// <summary>
            /// Returns a vector whose elements are the absolute values of each of the source vector's elements.
            /// </summary>
            /// <param name="value">The source vector.</param>
            /// <returns>The absolute value vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 Abs(Vector3 value)
            {
                return new Vector3(Math.Abs(value.X), Math.Abs(value.Y), Math.Abs(value.Z));
            }

            /// <summary>
            /// Returns a vector whose elements are the square root of each of the source vector's elements.
            /// </summary>
            /// <param name="value">The source vector.</param>
            /// <returns>The square root vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 SquareRoot(Vector3 value)
            {
                return new Vector3((Single)Math.Sqrt(value.X), (Single)Math.Sqrt(value.Y), (Single)Math.Sqrt(value.Z));
            }
            #endregion Public Static Methods

            #region Public Static Operators
            /// <summary>
            /// Adds two vectors together.
            /// </summary>
            /// <param name="left">The first source vector.</param>
            /// <param name="right">The second source vector.</param>
            /// <returns>The summed vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 operator +(Vector3 left, Vector3 right)
            {
                return new Vector3(left.X + right.X, left.Y + right.Y, left.Z + right.Z);
            }

            /// <summary>
            /// Subtracts the second vector from the first.
            /// </summary>
            /// <param name="left">The first source vector.</param>
            /// <param name="right">The second source vector.</param>
            /// <returns>The difference vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 operator -(Vector3 left, Vector3 right)
            {
                return new Vector3(left.X - right.X, left.Y - right.Y, left.Z - right.Z);
            }

            /// <summary>
            /// Multiplies two vectors together.
            /// </summary>
            /// <param name="left">The first source vector.</param>
            /// <param name="right">The second source vector.</param>
            /// <returns>The product vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 operator *(Vector3 left, Vector3 right)
            {
                return new Vector3(left.X * right.X, left.Y * right.Y, left.Z * right.Z);
            }

            /// <summary>
            /// Multiplies a vector by the given scalar.
            /// </summary>
            /// <param name="left">The source vector.</param>
            /// <param name="right">The scalar value.</param>
            /// <returns>The scaled vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 operator *(Vector3 left, Single right)
            {
                return left * new Vector3(right);
            }

            /// <summary>
            /// Multiplies a vector by the given scalar.
            /// </summary>
            /// <param name="left">The scalar value.</param>
            /// <param name="right">The source vector.</param>
            /// <returns>The scaled vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 operator *(Single left, Vector3 right)
            {
                return new Vector3(left) * right;
            }

            /// <summary>
            /// Divides the first vector by the second.
            /// </summary>
            /// <param name="left">The first source vector.</param>
            /// <param name="right">The second source vector.</param>
            /// <returns>The vector resulting from the division.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 operator /(Vector3 left, Vector3 right)
            {
                return new Vector3(left.X / right.X, left.Y / right.Y, left.Z / right.Z);
            }

            /// <summary>
            /// Divides the vector by the given scalar.
            /// </summary>
            /// <param name="value1">The source vector.</param>
            /// <param name="value2">The scalar value.</param>
            /// <returns>The result of the division.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 operator /(Vector3 value1, float value2)
            {
                float invDiv = 1.0f / value2;

                return new Vector3(
                    value1.X * invDiv,
                    value1.Y * invDiv,
                    value1.Z * invDiv);
            }

            /// <summary>
            /// Negates a given vector.
            /// </summary>
            /// <param name="value">The source vector.</param>
            /// <returns>The negated vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 operator -(Vector3 value)
            {
                return Zero - value;
            }

            /// <summary>
            /// Returns a boolean indicating whether the two given vectors are equal.
            /// </summary>
            /// <param name="left">The first vector to compare.</param>
            /// <param name="right">The second vector to compare.</param>
            /// <returns>True if the vectors are equal; False otherwise.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static bool operator ==(Vector3 left, Vector3 right)
            {
                return (left.X == right.X &&
                        left.Y == right.Y &&
                        left.Z == right.Z);
            }

            /// <summary>
            /// Returns a boolean indicating whether the two given vectors are not equal.
            /// </summary>
            /// <param name="left">The first vector to compare.</param>
            /// <param name="right">The second vector to compare.</param>
            /// <returns>True if the vectors are not equal; False if they are equal.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static bool operator !=(Vector3 left, Vector3 right)
            {
                return (left.X != right.X ||
                        left.Y != right.Y ||
                        left.Z != right.Z);
            }
            #endregion Public Static Operators

            #region Public Static Properties
            /// <summary>
            /// Returns the vector (0,0,0).
            /// </summary>
            public static Vector3 Zero { get { return new Vector3(); } }
            /// <summary>
            /// Returns the vector (1,1,1).
            /// </summary>
            public static Vector3 One { get { return new Vector3(1.0f, 1.0f, 1.0f); } }
            /// <summary>
            /// Returns the vector (1,0,0).
            /// </summary>
            public static Vector3 UnitX { get { return new Vector3(1.0f, 0.0f, 0.0f); } }
            /// <summary>
            /// Returns the vector (0,1,0).
            /// </summary>
            public static Vector3 UnitY { get { return new Vector3(0.0f, 1.0f, 0.0f); } }
            /// <summary>
            /// Returns the vector (0,0,1).
            /// </summary>
            public static Vector3 UnitZ { get { return new Vector3(0.0f, 0.0f, 1.0f); } }
            #endregion Public Static Properties

            #region Public Instance Methods

            /// <summary>
            /// Returns the hash code for this instance.
            /// </summary>
            /// <returns>The hash code.</returns>
            public override int GetHashCode()
            {
                int hash = this.X.GetHashCode();
                hash = ((hash << 5) + hash) ^ this.Y.GetHashCode();
                hash = ((hash << 5) + hash) ^ this.Z.GetHashCode();
                return hash;
            }

            /// <summary>
            /// Returns a boolean indicating whether the given Object is equal to this Vector3 instance.
            /// </summary>
            /// <param name="obj">The Object to compare against.</param>
            /// <returns>True if the Object is equal to this Vector3; False otherwise.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public override bool Equals(object obj)
            {
                if (!(obj is Vector3))
                    return false;
                return Equals((Vector3)obj);
            }

            /// <summary>
            /// Returns a String representing this Vector3 instance.
            /// </summary>
            /// <returns>The string representation.</returns>
            public override string ToString()
            {
                return ToString("G", CultureInfo.CurrentCulture);
            }

            /// <summary>
            /// Returns a String representing this Vector3 instance, using the specified format to format individual elements.
            /// </summary>
            /// <param name="format">The format of individual elements.</param>
            /// <returns>The string representation.</returns>
            public string ToString(string format)
            {
                return ToString(format, CultureInfo.CurrentCulture);
            }

            /// <summary>
            /// Returns a String representing this Vector3 instance, using the specified format to format individual elements 
            /// and the given IFormatProvider.
            /// </summary>
            /// <param name="format">The format of individual elements.</param>
            /// <param name="formatProvider">The format provider to use when formatting elements.</param>
            /// <returns>The string representation.</returns>
            public string ToString(string format, IFormatProvider formatProvider)
            {
                StringBuilder sb = new StringBuilder();
                string separator = NumberFormatInfo.GetInstance(formatProvider).NumberGroupSeparator;
                sb.Append('<');
                sb.Append(((IFormattable)this.X).ToString(format, formatProvider));
                sb.Append(separator);
                sb.Append(' ');
                sb.Append(((IFormattable)this.Y).ToString(format, formatProvider));
                sb.Append(separator);
                sb.Append(' ');
                sb.Append(((IFormattable)this.Z).ToString(format, formatProvider));
                sb.Append('>');
                return sb.ToString();
            }

            /// <summary>
            /// Returns the length of the vector.
            /// </summary>
            /// <returns>The vector's length.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public float Length()
            {
                float ls = Vector3.Dot(this, this);
                return (float)System.Math.Sqrt(ls);
            }

            /// <summary>
            /// Returns the length of the vector squared. This operation is cheaper than Length().
            /// </summary>
            /// <returns>The vector's length squared.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public float LengthSquared()
            {
                return Vector3.Dot(this, this);
            }
            #endregion Public Instance Methods

            #region Public Static Methods
            /// <summary>
            /// Returns the Euclidean distance between the two given points.
            /// </summary>
            /// <param name="value1">The first point.</param>
            /// <param name="value2">The second point.</param>
            /// <returns>The distance.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static float Distance(Vector3 value1, Vector3 value2)
            {
                Vector3 difference = value1 - value2;
                float ls = Vector3.Dot(difference, difference);
                return (float)System.Math.Sqrt(ls);
            }

            /// <summary>
            /// Returns the Euclidean distance squared between the two given points.
            /// </summary>
            /// <param name="value1">The first point.</param>
            /// <param name="value2">The second point.</param>
            /// <returns>The distance squared.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static float DistanceSquared(Vector3 value1, Vector3 value2)
            {
                Vector3 difference = value1 - value2;
                return Vector3.Dot(difference, difference);
            }

            /// <summary>
            /// Returns a vector with the same direction as the given vector, but with a length of 1.
            /// </summary>
            /// <param name="value">The vector to normalize.</param>
            /// <returns>The normalized vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 Normalize(Vector3 value)
            {
                float length = value.Length();
                return value / length;
            }

            /// <summary>
            /// Computes the cross product of two vectors.
            /// </summary>
            /// <param name="vector1">The first vector.</param>
            /// <param name="vector2">The second vector.</param>
            /// <returns>The cross product.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 Cross(Vector3 vector1, Vector3 vector2)
            {
                return new Vector3(
                    vector1.Y * vector2.Z - vector1.Z * vector2.Y,
                    vector1.Z * vector2.X - vector1.X * vector2.Z,
                    vector1.X * vector2.Y - vector1.Y * vector2.X);
            }

            /// <summary>
            /// Returns the reflection of a vector off a surface that has the specified normal.
            /// </summary>
            /// <param name="vector">The source vector.</param>
            /// <param name="normal">The normal of the surface being reflected off.</param>
            /// <returns>The reflected vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 Reflect(Vector3 vector, Vector3 normal)
            {
                float dot = Vector3.Dot(vector, normal);
                Vector3 temp = normal * dot * 2f;
                return vector - temp;
            }

            /// <summary>
            /// Restricts a vector between a min and max value.
            /// </summary>
            /// <param name="value1">The source vector.</param>
            /// <param name="min">The minimum value.</param>
            /// <param name="max">The maximum value.</param>
            /// <returns>The restricted vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 Clamp(Vector3 value1, Vector3 min, Vector3 max)
            {
                // This compare order is very important!!!
                // We must follow HLSL behavior in the case user specified min value is bigger than max value.

                float x = value1.X;
                x = (x > max.X) ? max.X : x;
                x = (x < min.X) ? min.X : x;

                float y = value1.Y;
                y = (y > max.Y) ? max.Y : y;
                y = (y < min.Y) ? min.Y : y;

                float z = value1.Z;
                z = (z > max.Z) ? max.Z : z;
                z = (z < min.Z) ? min.Z : z;

                return new Vector3(x, y, z);
            }

            /// <summary>
            /// Linearly interpolates between two vectors based on the given weighting.
            /// </summary>
            /// <param name="value1">The first source vector.</param>
            /// <param name="value2">The second source vector.</param>
            /// <param name="amount">Value between 0 and 1 indicating the weight of the second source vector.</param>
            /// <returns>The interpolated vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 Lerp(Vector3 value1, Vector3 value2, float amount)
            {
                Vector3 firstInfluence = value1 * (1f - amount);
                Vector3 secondInfluence = value2 * amount;
                return firstInfluence + secondInfluence;
            }

            #endregion Public Static Methods

            #region Public operator methods

            // All these methods should be inlined as they are implemented
            // over JIT intrinsics

            /// <summary>
            /// Adds two vectors together.
            /// </summary>
            /// <param name="left">The first source vector.</param>
            /// <param name="right">The second source vector.</param>
            /// <returns>The summed vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 Add(Vector3 left, Vector3 right)
            {
                return left + right;
            }

            /// <summary>
            /// Subtracts the second vector from the first.
            /// </summary>
            /// <param name="left">The first source vector.</param>
            /// <param name="right">The second source vector.</param>
            /// <returns>The difference vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 Subtract(Vector3 left, Vector3 right)
            {
                return left - right;
            }

            /// <summary>
            /// Multiplies two vectors together.
            /// </summary>
            /// <param name="left">The first source vector.</param>
            /// <param name="right">The second source vector.</param>
            /// <returns>The product vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 Multiply(Vector3 left, Vector3 right)
            {
                return left * right;
            }

            /// <summary>
            /// Multiplies a vector by the given scalar.
            /// </summary>
            /// <param name="left">The source vector.</param>
            /// <param name="right">The scalar value.</param>
            /// <returns>The scaled vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 Multiply(Vector3 left, Single right)
            {
                return left * right;
            }

            /// <summary>
            /// Multiplies a vector by the given scalar.
            /// </summary>
            /// <param name="left">The scalar value.</param>
            /// <param name="right">The source vector.</param>
            /// <returns>The scaled vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 Multiply(Single left, Vector3 right)
            {
                return left * right;
            }

            /// <summary>
            /// Divides the first vector by the second.
            /// </summary>
            /// <param name="left">The first source vector.</param>
            /// <param name="right">The second source vector.</param>
            /// <returns>The vector resulting from the division.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 Divide(Vector3 left, Vector3 right)
            {
                return left / right;
            }

            /// <summary>
            /// Divides the vector by the given scalar.
            /// </summary>
            /// <param name="left">The source vector.</param>
            /// <param name="divisor">The scalar value.</param>
            /// <returns>The result of the division.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 Divide(Vector3 left, Single divisor)
            {
                return left / divisor;
            }

            /// <summary>
            /// Negates a given vector.
            /// </summary>
            /// <param name="value">The source vector.</param>
            /// <returns>The negated vector.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static Vector3 Negate(Vector3 value)
            {
                return -value;
            }

            /// <summary>
            ///   Gets whether three points lie on the same line.
            /// </summary>
            /// 
            /// <param name="p1">The first point.</param>
            /// <param name="p2">The second point.</param>
            /// <param name="p3">The third point.</param>
            /// 
            /// <returns>True if there is a line passing through all
            ///  three points; false otherwise.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static bool Collinear(Vector3 p1, Vector3 p2, Vector3 p3, float eps = 1e-6f)
            {
                float x1m2 = p2.X - p1.X;
                float y1m2 = p2.Y - p1.Y;
                float z1m2 = p2.Z - p1.Z;

                float x2m3 = p3.X - p1.X;
                float y2m3 = p3.Y - p1.Y;
                float z2m3 = p3.Z - p1.Z;

                float x = y1m2 * z2m3 - z1m2 * y2m3;
                float y = z1m2 * x2m3 - x1m2 * z2m3;
                float z = x1m2 * y2m3 - y1m2 * x2m3;

                float norm = x * x + y * y + z * z;

                return norm < eps;
            }

            #endregion Public operator methods
        }
    }
}
