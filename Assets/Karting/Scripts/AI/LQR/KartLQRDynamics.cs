using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CenterSpace.NMath.Core;
using System;
using MathNet.Numerics.LinearAlgebra;

namespace KartGame.AI.LQR
{
    public abstract class KartLQRDynamics
    {
        public abstract Matrix<double> getA();
        public abstract Matrix<double> getB();
        public abstract int getXDim();
        public abstract int getUDim();
    }
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
                AMat[0, 2] = Math.Cos(initial[3])*dt;
                AMat[1, 2] = Math.Sin(initial[3])*dt;
            }
            return AMat;
        }

        public override Matrix<double> getB()
        {
            if (BMat == null)
            {
                BMat = CreateMatrix.Sparse<double>(xDim, uDim);
                BMat[2, 0] = dt;
                BMat[3, 1] = dt;
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
