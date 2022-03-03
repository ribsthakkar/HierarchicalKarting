using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CenterSpace.NMath.Core;
using System;
using MathNet.Numerics.LinearAlgebra;
using KartGame.AI.MPC;

namespace KartGame.AI.LQR
{
    /**
    * Construct the A and B matrices for the dynamics of an LQ problem
    **/
    public abstract class KartLQRDynamics
    {
        public abstract Matrix<double> getA();
        public abstract Matrix<double> getB();
        public abstract int getXDim();
        public abstract int getUDim();
    }

    /**
    * Linearized dynamics of the simplified bicycle model used in the MPC algorithm. The dynamics are linearized over the initial state.
    **/
    public class LinearizedBicycle : KartLQRDynamics
    {
        public const int xDim = 4;
        public const int uDim = 2;
        double dt;
        Vector<double> initial;
        Matrix<double> BMat = null;
        Matrix<double> AMat = null;

        public LinearizedBicycle(double dt, Vector<double> initial)
        {
            this.dt = dt;
            this.initial = initial.Clone();
        }

        public override Matrix<double> getA()
        {
            if (AMat == null)
            {
                AMat = CreateMatrix.SparseIdentity<double>(xDim);
                AMat[KartMPC.xIndex, KartMPC.vIndex] = Math.Cos(initial[KartMPC.hIndex]) * dt;
                AMat[KartMPC.zIndex, KartMPC.vIndex] = Math.Sin(initial[KartMPC.hIndex]) * dt;
                AMat[KartMPC.xIndex, KartMPC.hIndex] = -Math.Sin(initial[KartMPC.hIndex]) * dt * initial[KartMPC.vIndex];
                AMat[KartMPC.zIndex, KartMPC.hIndex] = Math.Cos(initial[KartMPC.hIndex]) * dt * initial[KartMPC.vIndex];
            }
            return AMat;
        }

        public override Matrix<double> getB()
        {
            if (BMat == null)
            {
                BMat = CreateMatrix.Sparse<double>(xDim, uDim);
                BMat[KartMPC.vIndex, 0] = dt;
                BMat[KartMPC.hIndex, 1] = dt;
            }
            return BMat;
        }

        public override int getXDim()
        {
            return xDim;
        }

        public override int getUDim()
        {
            return uDim;
        }
    }
}
