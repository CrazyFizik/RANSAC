using System;
using System.Collections.Generic;

namespace RANSAC
{
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
}
