using CenterSpace.NMath.Core;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KartGame.AI.MPC
{
    /**
    * Abstract/Interface of the Dynamics constraints used in the MPC calculations
    * Also check if the computed inputs satsify the dynamics
    **/
    public interface KartMPCDynamics
    {
        void AddDynamics(NonlinearProgrammingProblem problem);
        bool areInputsFeasible(DoubleVector x);
        DoubleVector nextX(DoubleVector x);
    }

    /**
    * Use the linearized bicycle dynamics.
    **/
    public class Bicycle : KartMPCDynamics
    {
        double dt;
        double aUpper;
        double aLower;
        double sUpper;
        double sLower;
        double vUpper;
        double lateralGsUpper;

        /**
        * Not the full Bicycle dyamics model with tire slip, but simplified with acceleration and yaw-rate inputs and using x,y,v,theta as state.
        * We also add bounds to the inputs in the state as well as maximum velocity and lateral Gs
        **/
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
            for(int t = 1; t < T; t++)
            {
                int i = t;
                // Piecewise Dynamics
                problem.AddNonlinearConstraint(
                    new NonlinearConstraint(problem.NumVariables, (vec) =>
                    (vec[KartMPC.xIndex * T + (i)] - vec[KartMPC.xIndex * T + (i - 1)]) - dt * vec[KartMPC.vIndex * T + (i - 1)] * Math.Cos(vec[KartMPC.hIndex * T + (i - 1)]),
                    ConstraintType.EqualTo));
                problem.AddNonlinearConstraint(
                    new NonlinearConstraint(problem.NumVariables, (vec) =>
                    (vec[KartMPC.zIndex * T + (i)] - vec[KartMPC.zIndex * T + (i - 1)]) - dt * vec[KartMPC.vIndex * T + (i - 1)] * Math.Sin(vec[KartMPC.hIndex * T + (i - 1)]),
                    ConstraintType.EqualTo));
                problem.AddNonlinearConstraint(
                    new NonlinearConstraint(problem.NumVariables, (vec) =>
                    (vec[KartMPC.hIndex * T + (i)] - vec[KartMPC.hIndex * T + (i - 1)]) - dt * vec[KartMPC.sIndex * T + (i - 1)],
                    ConstraintType.EqualTo));
                problem.AddNonlinearConstraint(
                    new NonlinearConstraint(problem.NumVariables, (vec) =>
                    (vec[KartMPC.vIndex * T + (i)] - vec[KartMPC.vIndex * T + (i - 1)]) - dt * vec[KartMPC.aIndex * T + (i - 1)],
                    ConstraintType.EqualTo));

                // Control input Bounds
                problem.AddBounds(KartMPC.sIndex * T + (i), sLower, sUpper);
                problem.AddBounds(KartMPC.aIndex * T + (i), aLower, aUpper);

                // State Bounds
                problem.AddBounds(KartMPC.vIndex * T + (i), 0, vUpper);

                // Dynamical Constraints
                //problem.AddNonlinearConstraint(new NonlinearConstraint(problem.NumVariables, (vec) =>
                //{
                //    double lateralGs = ((vec[KartMPC.sIndex * T + (i)] - vec[KartMPC.sIndex * T + (i - 1)]) / dt) / 9.81;
                //    return lateralGsUpper - lateralGs;
                //}, ConstraintType.GreaterThanOrEqualTo));

            }
        }

        public bool areInputsFeasible(DoubleVector vec)
        {
            int T = vec.Length / (KartMPC.xDim + KartMPC.uDim);
            for (int i = 1; i < T; i++)
            {
                if (vec[KartMPC.sIndex * T + (i)] > aUpper || vec[KartMPC.sIndex * T + (i)] < sLower)
                {
                    return false;
                }
                if (vec[KartMPC.aIndex * T + (i)] > aUpper || vec[KartMPC.aIndex * T + (i)] < aLower)
                {
                    return false;
                }
                if (Math.Abs((vec[KartMPC.xIndex * T + (i)] - vec[KartMPC.xIndex * T + (i - 1)]) - dt * vec[KartMPC.vIndex * T + (i - 1)] * Math.Cos(vec[KartMPC.hIndex * T + (i - 1)])) >= 1e-3)
                {
                    Debug.Log("Broke x dynamics: " + Math.Abs((vec[KartMPC.xIndex * T + (i)] - vec[KartMPC.xIndex * T + (i - 1)]) - dt * vec[KartMPC.vIndex * T + (i - 1)] * Math.Cos(vec[KartMPC.hIndex * T + (i - 1)])));
                    return false;
                }
                if (Math.Abs((vec[KartMPC.zIndex * T + (i)] - vec[KartMPC.zIndex * T + (i - 1)]) - dt * vec[KartMPC.vIndex * T + (i - 1)] * Math.Sin(vec[KartMPC.hIndex * T + (i - 1)])) >= 1e-3)
                {
                    Debug.Log("Broke z dynamics: " + Math.Abs((vec[KartMPC.zIndex * T + (i)] - vec[KartMPC.zIndex * T + (i - 1)]) - dt * vec[KartMPC.vIndex * T + (i - 1)] * Math.Sin(vec[KartMPC.hIndex * T + (i - 1)])));
                    return false;
                }
                if (Math.Abs((vec[KartMPC.hIndex * T + (i)] - vec[KartMPC.hIndex * T + (i - 1)]) - dt * vec[KartMPC.sIndex * T + (i - 1)]) >= 1e-3)
                {
                    Debug.Log("Broke heading dyanmics" + Math.Abs((vec[KartMPC.hIndex * T + (i)] - vec[KartMPC.hIndex * T + (i - 1)]) - dt * vec[KartMPC.sIndex * T + (i - 1)]));
                    return false;
                }
                if (Math.Abs((vec[KartMPC.vIndex * T + (i)] - vec[KartMPC.vIndex * T + (i - 1)]) - dt * vec[KartMPC.aIndex * T + (i - 1)]) >= 1e-3)
                {
                    Debug.Log("Broke velocity dynamics: " + Math.Abs((vec[KartMPC.vIndex * T + (i)] - vec[KartMPC.vIndex * T + (i - 1)]) - dt * vec[KartMPC.aIndex * T + (i - 1)]));
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