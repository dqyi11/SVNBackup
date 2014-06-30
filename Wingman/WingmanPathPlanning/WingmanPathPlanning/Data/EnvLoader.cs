using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Xml;
using WingmanPathPlanning.Hexagonal;

namespace WingmanPathPlanning.Data
{
    class Gaussian
    {
        public double meanX;
        public double meanY;
        public double varianceX;
        public double varianceY;
        public double rho;

        public Gaussian(double mX, double mY, double varX, double varY, double P)
        {
            meanX = mX;
            meanY = mY;
            varianceX = varX;
            varianceY = varY;
            rho = P;
        }
    }

    class EnvLoader
    {
        protected XmlDocument _xDoc;
        List<Gaussian> gaussianList;

        public EnvLoader()
        {
            _xDoc = new XmlDocument();
            gaussianList = new List<Gaussian>();
        }
        
        public void Load(string filename)
        {
            _xDoc.Load(filename);
            XmlNode envNode = _xDoc.SelectSingleNode("Environment");
            gaussianList.Clear();

            XmlNodeList gaussianNodeList = envNode.SelectNodes("Gaussian");
            if (gaussianNodeList != null)
            {
                foreach (XmlNode gaussianNode in gaussianNodeList)
                {
                    string meanX = gaussianNode.Attributes["meanX"].Value;
                    string meanY = gaussianNode.Attributes["meanY"].Value;
                    string varX = gaussianNode.Attributes["varX"].Value;
                    string varY = gaussianNode.Attributes["varY"].Value;
                    string highProb = gaussianNode.Attributes["rho"].Value;

                    Gaussian gaussian = new Gaussian(Double.Parse(meanX), Double.Parse(meanY),
                        Double.Parse(varX), Double.Parse(varY), Double.Parse(highProb));
                    gaussianList.Add(gaussian);
                }
            }            
        }

        public double[,] CalcVal(HexagonalMap map)
        {
            double[,] newEntropy = (double[,])map.GetMapStateMgr().GetEntropy().Clone();

            //how to caculate a point?
            // check GMM one by one , take the max

            for (int i = 0; i < map.Height; i++)
            {
                for (int j = 0; j < map.Width; j++)
                {
                    Hex hex = map.GetHex(i, j);

                    double maxVal = 0.0;
                    List<Gaussian>.Enumerator e = gaussianList.GetEnumerator();
                    while (e.MoveNext())
                    {
                        double gVal = 0;
                        int pointNum = hex.Points.Length;
                        double centerX = 0.0;
                        double centerY = 0.0;
                        for(int k=0;k<pointNum;k++)
                        {
                            centerX += hex.Points[k].X;
                            centerY += hex.Points[k].Y;
                            
                        }

                        centerX = centerX / (double)pointNum;
                        centerY = centerY / (double)pointNum;
                        gVal = CalcGaussian(e.Current, centerX, centerY);
                        
                        if (maxVal < gVal)
                        {
                            maxVal = gVal;
                        }
                    }
                    newEntropy[i, j] = maxVal;
                }
            }

            return newEntropy;
        }

        public double CalcGaussian(Gaussian gaussian, double posX, double posY)
        {
            double val = 0.0;

            double varX = gaussian.varianceX;
            double varY = gaussian.varianceY;
            double muX = gaussian.meanX;
            double muY = gaussian.meanY;
            double rho = gaussian.rho;

            val = GetBivariateGuassian(muX, varX, muY, varY, posX, posY, rho);

            // gmaConsole.WriteLine("X:" + posX + ",Y:" + posY + ",val:" + val);
            return val;
        }

        public static double GetBivariateGuassian(double muX, double sigmaX, double muY, double sigmaY, double x, double y, double rho = 0)
        {
            var sigmaXSquared = Math.Pow(sigmaX, 2);
            var sigmaYSquared = Math.Pow(sigmaY, 2);

            var dX = x - muX;
            var dY = y - muY;

            var exponent = -0.5;
            var normaliser = 2 * Math.PI * sigmaX * sigmaY;
            if (rho != 0)
            {
                normaliser *= Math.Sqrt(1 - Math.Pow(rho, 2));
                exponent /= 1 - Math.Pow(rho, 2);
            }

            var sum = Math.Pow(dX, 2) / sigmaXSquared;
            sum += Math.Pow(dY, 2) / sigmaYSquared;
            sum -= 2 * rho * dX * dY / (sigmaX * sigmaY);

            exponent *= sum;

            // return Math.Exp(exponent) / normaliser;
            return Math.Exp(exponent);
        }
    }
}
