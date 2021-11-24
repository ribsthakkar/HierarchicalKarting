using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CenterSpace.NMath.Core;
using System;

namespace KartGame.AI.LQR
{
    public abstract class KartLQRDynamics: MonoBehaviour
    {
        public abstract DoubleMatrix getA(DoubleVector initial);
        public abstract DoubleMatrix getB(DoubleVector initial);
        public abstract int getXDim();
        public abstract int getUDim();
    }
    public class LinearizedBicycle : KartLQRDynamics
    {
        public const int xDim = 4;
        public const int uDim = 2;
        double dt;
        double aUpper;
        double aLower;
        double sUpper;
        double sLower;
        double vUpper;
        double lateralGsUpper;

        public LinearizedBicycle(double dt, double aUpper, double aLower, double sUpper, double sLower, double vUpper, double lateralGsUpper)
        {
            this.dt = dt;
            this.aUpper = aUpper;
            this.aLower = aLower;
            this.sUpper = sUpper;
            this.sLower = sLower;
            this.vUpper = vUpper;
            this.lateralGsUpper = lateralGsUpper;
        }

        public override DoubleMatrix getA(DoubleVector initial)
        {
            var AMat = DoubleMatrix.Identity(xDim);
            AMat[0, 2] = Math.Cos(initial[2]);
            AMat[1, 2] = Math.Sin(initial[2]);
            return AMat;
        }

        public override DoubleMatrix getB(DoubleVector initial)
        {
            var BMat = new DoubleMatrix(xDim, uDim);
            BMat[2, 0] = dt;
            BMat[3, 1] = dt;
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
