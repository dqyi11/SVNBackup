using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using DataMgr;
using SVM;

namespace KernelMachine
{
    class SVMTester
    {
        IrisDataMgr _dataMgr;
        Random _rnd;
        Model _model;

        public SVMTester(IrisDataMgr dataMgr)
        {
            _dataMgr = dataMgr;
            _rnd = new Random();
        }

        public void Train()
        {
            int datasetSize = _dataMgr.count;
            int dimension = _dataMgr.inputNum;
            Node[][] inputs = new Node[datasetSize][];
            double[] labels = new double[datasetSize];

            int i = 0;
            int j = 0;
            for (i = 0; i < datasetSize; i++)
            {
                int index = i; // _rnd.Next(datasetSize - 1);
                inputs[i] = new Node[dimension];
                double[] input = _dataMgr.GetInputData(index);
                for (j = 0; j < dimension; j++)
                {
                    inputs[i][j] = new Node();
                    inputs[i][j].Index = j;
                    inputs[i][j].Value = input[j];
                }

                if (_dataMgr.GetLabelData(index) < 0.5)
                {
                    labels[i] = -1;
                }
                else
                {
                    labels[i] = 1;
                }
            }

            Problem problem = new Problem(datasetSize, labels, inputs, dimension);

            //RangeTransform range = RangeTransform.Compute(problem);
            //problem = range.Scale(problem);

            Parameter param = new Parameter();
            param.C = 2;
            param.Gamma = .5;
            //param.KernelType = KernelType.RBF;
            //param.SvmType = SvmType.C_SVC;

            _model = new Model();
            _model.Parameter = param;

            bool[] nonzero = new bool[datasetSize];

            double[] alpha = new double[problem.Count];

            int l = problem.Count;
            double[] Minus_ones = new double[l];
            sbyte[] y = new sbyte[l];

            for (i = 0; i < l; i++)
            {
                alpha[i] = 0;
                Minus_ones[i] = -1;
                if (problem.Y[i] > 0) y[i] = +1; else y[i] = -1;
            }

            Solver s = new Solver();
            Solver.SolutionInfo si = new Solver.SolutionInfo();
            s.Solve(l, new SVC_Q(problem, param, y), Minus_ones, y,
                alpha, param.C, param.C, param.EPS, si, param.Shrinking);

            double sum_alpha = 0;
            for (i = 0; i < l; i++)
                sum_alpha += alpha[i];

            for (i = 0; i < l; i++)
                alpha[i] *= y[i];

            Procedures.decision_function f = new Procedures.decision_function();
            f.alpha = alpha;
            f.rho = si.rho;

            // build output

            _model.NumberOfClasses = 2;
            _model.ClassLabels = new int[2];
            _model.ClassLabels[0] = 1;
            _model.ClassLabels[1] = -1;

            _model.Rho = new double[1];
            _model.Rho[0] = f.rho;

            _model.NumberOfSVPerClass = new int[2];
            int posCnt = 0, negCnt = 0;
            for (i = 0; i < datasetSize; i++)
            {
                if (labels[i] > 0)
                {
                    posCnt++;
                }
                else
                {
                    negCnt++;
                }
            }
            _model.NumberOfSVPerClass[0] = posCnt;
            _model.NumberOfSVPerClass[1] = negCnt;

            _model.SupportVectorCount = datasetSize;
            _model.SupportVectors = new Node[datasetSize][];
            _model.SupportVectorCoefficients = new double[1][];
            _model.SupportVectorCoefficients[0] = new double[datasetSize];
            int p = 0;
            for (i = 0; i < datasetSize; i++)
            {
                _model.SupportVectors[p++] = inputs[i];
                _model.SupportVectorCoefficients[0][i] = f.alpha[i];
            }

            //_model = Training.Train(problem, param);
        }

        public void OneClassTrain()
        {
            int datasetSize = _dataMgr.count;
            int dimension = _dataMgr.inputNum;
            Node[][] inputs = new Node[datasetSize][];
            double[] labels = new double[datasetSize];

            int i = 0;
            int j = 0;
            for (i = 0; i < datasetSize; i++)
            {
                int index = i; // _rnd.Next(datasetSize - 1);
                inputs[i] = new Node [dimension];
                double [] input = _dataMgr.GetInputData(index);
                for (j = 0; j < dimension; j++)
                {
                    inputs[i][j] = new Node();
                    inputs[i][j].Index = j;
                    inputs[i][j].Value = input[j];
                }
                
                if(_dataMgr.GetLabelData(index)<0.5)
                {
                    labels[i] = -1;
                }
                else
                {
                    labels[i] = 1;
                }
            }

            Problem problem = new Problem(datasetSize, labels, inputs, dimension);

            //RangeTransform range = RangeTransform.Compute(problem);
            //problem = range.Scale(problem);

            Parameter param = new Parameter();
            param.C = 2;
            param.Gamma = .5;
            //param.KernelType = KernelType.RBF;
            //param.SvmType = SvmType.ONE_CLASS;
            //param.Nu = 0.5;

            /*
            _model = new Model();
            _model.Parameter = param;
            
            _model.NumberOfClasses = 2;                
            _model.ClassLabels = null;
            _model.NumberOfSVPerClass = null;
            _model.PairwiseProbabilityA = null; 
            _model.PairwiseProbabilityB = null;
            _model.SupportVectorCoefficients = new double[1][];

            double[] alpha = new double[datasetSize];
            Solver.SolutionInfo si = new Solver.SolutionInfo();

            int l = problem.Count;
                    
            double[] zeros = new double[l];
            sbyte[] ones = new sbyte[l];
            

            int n = (int)(param.Nu * problem.Count);	// # of alpha's at upper bound

            for (i = 0; i < n; i++)
                alpha[i] = 1;
            if (n < problem.Count)
                alpha[n] = param.Nu * problem.Count - n;
            for (i = n + 1; i < l; i++)
                alpha[i] = 0;

            for (i = 0; i < l; i++)
            {
                zeros[i] = 0;
                ones[i] = 1;
            } 
             
            Solver s = new Solver();
            s.Solve(l, new ONE_CLASS_Q(problem, param), zeros, ones, alpha, 1.0, 1.0, param.EPS, si, param.Shrinking);
             
            Procedures.decision_function f = new Procedures.decision_function();
            f.alpha = alpha;
            f.rho = si.rho;
             
            _model.Rho = new double[1];
            _model.Rho[0] = f.rho;

            int nSV = 0;
            for (i = 0; i < problem.Count; i++)
                if (Math.Abs(f.alpha[i]) > 0) ++nSV;
            _model.SupportVectorCount = nSV;
            _model.SupportVectors = new Node[nSV][];
            _model.SupportVectorCoefficients[0] = new double[nSV];
            
            j = 0;
            for (i = 0; i < problem.Count; i++)
            {
                if (Math.Abs(f.alpha[i]) > 0)
                {
                    _model.SupportVectors[j] = problem.X[i];
                    _model.SupportVectorCoefficients[0][j] = f.alpha[i];
                    ++j;
                }
            }
            */
            

            _model = Training.Train(problem, param);
            

            
        }

        public double Predict(double[] input)
        {
            int dimension = input.Length;
            Node[] inputNode = new Node[dimension];
            int i;
            for (i = 0; i < dimension; i++)
            {
                inputNode[i] = new Node();
                inputNode[i].Index = i;
                inputNode[i].Value = input[i];
            }

            double dec_values = new double();

            int l = _model.SupportVectorCount;

            double[] kvalue = new double[l];
            for (i = 0; i < l; i++)
                kvalue[i] = Kernel.KernelFunction(inputNode, _model.SupportVectors[i], _model.Parameter);

            for (i = 0; i < l; i++)
            {
                dec_values += _model.SupportVectorCoefficients[0][i] * kvalue[i];
            }

            dec_values -= _model.Rho[0];

            int vote_Max_idx = 0;
            if (dec_values > 0)
            {
                vote_Max_idx = 0;
            }
            else
            {
                vote_Max_idx = 1;
            }
            return _model.ClassLabels[vote_Max_idx];

        }

        public void Test()
        {
            int datasetSize = _dataMgr.count;
            int dimension = _dataMgr.inputNum;
            
            Node[][] inputs = new Node[datasetSize][];
            int[] labels = new int[datasetSize];

            for (int i = 0; i < datasetSize; i++)
            {
                int index = _rnd.Next(datasetSize - 1);
                inputs[i] = new Node[dimension];
                double[] input = _dataMgr.GetInputData(index);
                for (int j = 0; j < dimension; j++)
                {
                    inputs[i][j] = new Node();
                    inputs[i][j].Index = j;
                    inputs[i][j].Value = input[j];
                }

                if (_dataMgr.GetLabelData(index) < 0.5)
                {
                    labels[i] = -1;
                }
                else
                {
                    labels[i] = 1;
                }
            }

            int successCnt = 0;
            for (int i = 0; i < datasetSize; i++)
            {
                double value = Prediction.Predict(_model, inputs[i]);
              

                if ((int)value == (int)labels[i])
                {
                    successCnt++;
                }
                Console.WriteLine("compare " + labels[i] + " and " + value);
            }

            Console.WriteLine("success rate:" + (double)successCnt / (double)datasetSize);
           
        }

        public void Test2()
        {
            int datasetSize = _dataMgr.count;
            int dimension = _dataMgr.inputNum;

            double[][] inputs = new double[datasetSize][];
            int[] labels = new int[datasetSize];

            for (int i = 0; i < datasetSize; i++)
            {
                int index = _rnd.Next(datasetSize - 1);
                inputs[i] = _dataMgr.GetInputData(index);
            

                if (_dataMgr.GetLabelData(index) < 0.5)
                {
                    labels[i] = -1;
                }
                else
                {
                    labels[i] = 1;
                }
            }

            int successCnt = 0;
            for (int i = 0; i < datasetSize; i++)
            {
                double value = Predict(inputs[i]);


                if ((int)value == (int)labels[i])
                {
                    successCnt++;
                }
                Console.WriteLine("compare " + labels[i] + " and " + value);
            }

            Console.WriteLine("success rate:" + (double)successCnt / (double)datasetSize);

        }
    }

}
