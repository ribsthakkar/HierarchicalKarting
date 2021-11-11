using CenterSpace.NMath.Core;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KartGame.AI.MPC
{
    public interface KartMPCDynamics
    {
        void AddDynamics(NonlinearProgrammingProblem problem);
        bool areInputsFeasible(DoubleVector x);
        DoubleVector nextX(DoubleVector x);
    }

    public class Bicycle : KartMPCDynamics
    {
        double dt;
        double aUpper;
        double aLower;
        double sUpper;
        double sLower;
        double vUpper;
        double lateralGsUpper;

        public Bicycle(double dt, double aUpper, double aLower, double sUpper, double sLower, double vUpper, double lateralGsUpper)
        {
            this.dt = dt;
            this.aUpper = aUpper;
            this.aLower = aLower;
            this.sUpper = sUpper;
            this.sLower = sLower;
            this.vUpper = vUpper;
            this.lateralGsUpper = lateralGsUpper;
        }

        public void AddDynamics(NonlinearProgrammingProblem problem)
        {
            int T = problem.NumVariables / (KartMPC.xDim + KartMPC.uDim);
            for(int i = 1; i < T; i++)
            {
                // Piecewise Dynamics
                problem.AddNonlinearConstraint(
                    new NonlinearConstraint(problem.NumVariables, (vec) =>
                    (vec[KartMPC.xIndex*T + (i)] - vec[KartMPC.xIndex*T + (i-1)]) - dt*vec[KartMPC.vIndex*T + (i-1)] * Math.Cos(vec[KartMPC.hIndex * T + (i - 1)]), 
                    ConstraintType.EqualTo));
                problem.AddNonlinearConstraint(
                    new NonlinearConstraint(problem.NumVariables, (vec) =>
                    (vec[KartMPC.zIndex * T + (i)] - vec[KartMPC.zIndex * T + (i - 1)]) - dt*vec[KartMPC.vIndex * T + (i - 1)] * Math.Sin(vec[KartMPC.hIndex * T + (i - 1)]),
                    ConstraintType.EqualTo));
                problem.AddNonlinearConstraint(
                    new NonlinearConstraint(problem.NumVariables, (vec) =>
                    (vec[KartMPC.hIndex * T + (i)] - vec[KartMPC.hIndex * T + (i - 1)]) - dt*vec[KartMPC.sIndex * T + (i - 1)],
                    ConstraintType.EqualTo));
                problem.AddNonlinearConstraint(
                    new NonlinearConstraint(problem.NumVariables, (vec) =>
                    (vec[KartMPC.vIndex * T + (i)] - vec[KartMPC.vIndex * T + (i - 1)]) - dt*vec[KartMPC.aIndex * T + (i - 1)],
                    ConstraintType.EqualTo));

                // Control input Bounds
                problem.AddBounds(KartMPC.sIndex * T + (i - 1), sLower, sUpper);
                problem.AddBounds(KartMPC.aIndex * T + (i - 1), aLower, aUpper);

                // State Bounds
                problem.AddBounds(KartMPC.vIndex * T + (i - 1), 0, vUpper);

                // Dynamical Constraints
                problem.AddNonlinearConstraint(new NonlinearConstraint(problem.NumVariables, (vec) =>
                {
                    double lateralGs = ((vec[KartMPC.sIndex * T + (i)] - vec[KartMPC.sIndex * T + (i - 1)])/dt)/9.81;
                    return lateralGsUpper - lateralGs;
                }, ConstraintType.GreaterThanOrEqualTo));

            }
        }

        public bool areInputsFeasible(DoubleVector x)
        {
            int T = x.Length / (KartMPC.xDim + KartMPC.uDim);
            for (int i = 1; i < T; i++)
            {
                if (x[KartMPC.sIndex * T + (i)] > aUpper || x[KartMPC.sIndex * T + (i)] < sLower)
                {
                    return false;
                }
                if (x[KartMPC.aIndex * T + (i)] > aUpper || x[KartMPC.aIndex * T + (i)] < aLower)
                {
                    return false;
                }
            }
            return true;
        }

        public DoubleVector nextX(DoubleVector x)
        {
            throw new System.NotImplementedException();
        }
    }
}