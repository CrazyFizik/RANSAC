using System;
using System.IO;
using System.Globalization;
using System.Collections.Generic;
using RANSAC;

namespace RANSAC.Console
{
    class Program
    {
        public const string DATA_PATH = "Data";
        public const string INPUT_FILE = "Data\\input.txt";
        public const string OUTPUT_FILE = "Data\\output.txt";

        static void Main(string[] args)
        {
            //DemoStaticTest();
            DemoTest();
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
            while((line = reader.ReadLine())!=null)
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

        static void DemoTest()
        {
            var dir = Directory.GetCurrentDirectory();
            var rp = ReadPlaneFromFile(dir, INPUT_FILE);
            System.Console.WriteLine(rp.BestModel.ToString());
            SavePlaneToFile(rp, dir, OUTPUT_FILE);
            System.Console.ReadKey();
        }

        static void DemoStaticTest()
        {
            RansacPlane rp = new RansacPlane();

            System.Console.Write("A1: ");
            rp.Estimate(RansacPlane.GetTestA1());
            System.Console.WriteLine(rp.BestModel.ToString());

            System.Console.Write("A2: ");
            rp.Estimate(RansacPlane.GetTestA2());
            System.Console.WriteLine(rp.BestModel.ToString());

            System.Console.Write("A3: ");
            rp.Estimate(RansacPlane.GetTestA3());
            System.Console.WriteLine(rp.BestModel.ToString());

            System.Console.ReadKey();
        }
    }
}
