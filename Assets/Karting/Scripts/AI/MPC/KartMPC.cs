using CenterSpace.NMath.Core;
using KartGame.AI;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

namespace KartGame.AI.MPC
{
    public class KartMPC : MonoBehaviour
    {
        public const int xDim = 4;
        public const int uDim = 2;
        public const int xIndex = 0;
        public const int zIndex = 1;
        public const int vIndex = 2;
        public const int hIndex = 3;
        public const int aIndex = 4;
        public const int sIndex = 5;


        public static List<DoubleVector> solveGame(List<KartMPCDynamics> dynamics, List<List<KartMPCCosts>> individualCosts, List<List<KartMPCConstraints>> individualConstraints, List<List<CoupledKartMPCCosts>> coupledCosts, List<List<CoupledKartMPCConstraints>> coupledConstraints, List<DoubleVector> initial, int steps, int maxIBRIterations=2)
        {
            int N = individualCosts.Count;
            List<DoubleVector> last_trajectory = new List<DoubleVector>();
            for (int ii = 0; ii < N; ii++)
            {
                last_trajectory.Add(solveOptimalControl(dynamics[ii], individualConstraints[ii], individualCosts[ii], initial[ii], steps));
            }
            print("Finished initial trajectory");
            for(int iter = 0; iter < maxIBRIterations; iter++)
            {
                for (int ii = 0; ii < N; ii++)
                {
                    foreach (CoupledKartMPCCosts c in coupledCosts[ii])
                    {
                        c.other = last_trajectory[c.otherIdx];
                    }
                    foreach (CoupledKartMPCConstraints c in coupledConstraints[ii])
                    {
                        c.other = last_trajectory[c.otherIdx];
                    }

                    DoubleVector newSol = solveOptimalControl(dynamics[ii], individualConstraints[ii].Concat(coupledConstraints[ii]).ToList(), individualCosts[ii].Concat(coupledCosts[ii]).ToList(), initial[ii], steps);
                    last_trajectory[ii] = newSol;
                }
            }
            return last_trajectory;
        }

        public static DoubleVector solveOptimalControl(KartMPCDynamics dynamics, List<KartMPCConstraints> constraints, List<KartMPCCosts> costs, DoubleVector initial, int steps)
        {
            int totalDim = (xDim + uDim) * steps;
            // Define Objective function
            Func<DoubleVector, double> objective = (DoubleVector x) =>
            {
                return costs.Sum((c) => c.Evaluate(x));
            };

            var problem = new NonlinearProgrammingProblem(totalDim, objective);
            var x0 = new DoubleVector(totalDim);
            //Set initial variable constraints

            for (int i = 0; i < totalDim; i += steps)
            {
                problem.AddBounds(i, initial[i], initial[i]);
                x0[i] = initial[i];
            }


            // Add dynamic constraints
            dynamics.AddDynamics(problem);

            // Add remaining Constraints
            foreach (KartMPCConstraints c in constraints)
            {
                c.AddConstraints(problem);
            }

            var solver = new ActiveSetLineSearchSQP(1e-4);
            bool succcess = solver.Solve(problem, x0);
            // print(solver.OptimalX.ToString());
            print(solver.SolverTerminationStatus);
            DoubleMatrix m = new DoubleMatrix(solver.OptimalX).Resize(solver.OptimalX.Length / 6, 6).Transpose();
            print(objective(solver.OptimalX) + " " + m.ToString());
            //var solverParams = new StochasticHillClimbingParameters { Minimize = true, TimeLimitMilliSeconds = 10000, Presolve=true };
            //var solver = new StochasticHillClimbingSolver();
            //solver.Solve(problem, x0, solverParams);
            //print(solver.OptimalX);
            return solver.OptimalX;
        }
    }
}
