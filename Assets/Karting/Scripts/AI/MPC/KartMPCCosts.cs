using CenterSpace.NMath.Core;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

namespace KartGame.AI.MPC
{
    public interface KartMPCCosts
    {
        double Evaluate(DoubleVector m);
    }

    public abstract class CoupledKartMPCCosts : KartMPCCosts
    {
        public DoubleVector other;
        public int otherIdx;

        public abstract double Evaluate(DoubleVector m);
    }


    public class WaypointCost: KartMPCCosts
    {
        double position_weight;
        double speed_weight;
        double x;
        double z;
        double speed;

        public WaypointCost(double position_weight, double speed_weight, double x, double z, double speed)
        {
            this.position_weight = position_weight;
            this.speed_weight = speed_weight;
            this.x = x;
            this.z = z;
            this.speed = speed;
        }

        public double Evaluate(DoubleVector v)
        {
            int T = v.Length / (KartMPC.xDim + KartMPC.uDim);
            double minDist = 1000000.0;
            double distCost = 0.0;
            double speedCost = 0.0;
            for(int j = 0; j < T; j++)
            {
                double dist = Math.Pow((v[KartMPC.xIndex*T + j] - x), 2) + Math.Pow((v[KartMPC.zIndex*T + j] - z), 2);
                if (dist < minDist)
                {
                    distCost = 0.5 * position_weight * dist;
                    speedCost = 0.5 * speed_weight * Math.Pow(speed - v[KartMPC.vIndex*T + j], 2);
                }
            }
            return  distCost + speedCost;
        }
    }

    public class DistanceFromCenterCost : KartMPCCosts
    {
        List<Vector2> centerPoints;
        int initialIndex;
        int finalIndex;
        double maxDist;
        double weight;

        public DistanceFromCenterCost(List<Vector2> centerPoints, int initialIndex, int finalIndex, double maxDist, double weight)
        {
            this.centerPoints = centerPoints;
            this.initialIndex = initialIndex;
            this.finalIndex = finalIndex;
            this.maxDist = maxDist;
            this.weight = weight;
        }
        public double Evaluate(DoubleVector x)
        {
            int T = x.Length / (KartMPC.xDim + KartMPC.uDim);
            double cost = 0;
            for (int i = 1; i < T; i++)
            {
                double minDist = 1000*1000;
                for (int j = initialIndex; j < finalIndex; j++)
                {
                    minDist = Math.Min(Math.Pow(centerPoints[j % centerPoints.Count].x - x[KartMPC.xIndex * T + (i)], 2) + Math.Pow(x[KartMPC.zIndex * T + (i)] - centerPoints[j % centerPoints.Count].y, 2), minDist);
                }
                cost += minDist <= maxDist*maxDist ? 0 : weight * Math.Sqrt(minDist-maxDist*maxDist);
            }
            return cost;
        }
    }

    public class ForwardProgressReward : KartMPCCosts
    {
        List<Vector2> centerPoints;
        int initialIndex;
        int finalIndex;
        double weight;

        public ForwardProgressReward(List<Vector2> centerPoints, int initialIndex, int finalIndex, double weight)
        {
            this.centerPoints = centerPoints;
            this.initialIndex = initialIndex;
            this.finalIndex = finalIndex;
            this.weight = weight;
        }
        public double Evaluate(DoubleVector x)
        {
            int T = x.Length / (KartMPC.xDim + KartMPC.uDim);
            int finalT = T - 1;
            double minDist = 1000 * 1000;
            int bestProgress = initialIndex;
            for (int j = initialIndex; j < finalIndex; j++)
            {
                if (Math.Pow(centerPoints[j % centerPoints.Count].x - x[KartMPC.xIndex * T + (finalT)], 2) + Math.Pow(x[KartMPC.zIndex * T + (finalT)] - centerPoints[j % centerPoints.Count].y, 2) <= minDist)
                {
                    minDist = Math.Pow(centerPoints[j % centerPoints.Count].x - x[KartMPC.xIndex * T + (finalT)], 2) + Math.Pow(x[KartMPC.zIndex * T + (finalT)] - centerPoints[j % centerPoints.Count].y, 2);
                    bestProgress = j;
                }
            }
            // UnityEngine.Debug.Log("obj:" + (-bestProgress*weight) + " x " + x.ToString());
            return -bestProgress * weight;
        }
    }

    public class DistanceTraveledReward : KartMPCCosts
    {
        double weight;
        double dt;

        public DistanceTraveledReward(double dt, double weight)
        {
            this.dt = dt;
            this.weight = weight;
        }
        public double Evaluate(DoubleVector x)
        {
            int T = x.Length / (KartMPC.xDim + KartMPC.uDim);
            int finalT = T - 1;
            double dist = 0.0;
            for (int t = 0; t < T; t++)
            {
                dist += x[KartMPC.vIndex * T + t] * dt;
            }
            // UnityEngine.Debug.Log("obj:" + (-bestProgress*weight) + " x " + x.ToString());
            return -dist * weight;
        }
    }

    public class CoupledDistanceCost : CoupledKartMPCCosts
    {
        double minDist;
        double weight;

        public CoupledDistanceCost(double minDist, double weight, int otherIdx)
        {
            this.otherIdx = otherIdx;
            this.minDist = minDist;
            this.weight = weight;
        }
        public override double Evaluate(DoubleVector x)
        {
            int T = x.Length / (KartMPC.xDim + KartMPC.uDim);
            double cost = 0;
            for (int i = 1; i < T; i++)
            {
                double dist2 = Math.Pow(x[KartMPC.xIndex * T + (i)] - other[KartMPC.xIndex * T + (i)], 2) + Math.Pow(x[KartMPC.zIndex * T + (i)] - other[KartMPC.zIndex * T + (i)], 2);
                cost += dist2 >= minDist*minDist ? 0 : 0.5 * weight * dist2;
            }
            return cost;
        }
    }

    public class CoupledProgressReward: CoupledKartMPCCosts
    {
        List<Vector2> centerPoints;
        int initialIndex;
        int finalIndex;
        double weight;

        public CoupledProgressReward(List<Vector2> centerPoints, int initialIndex, int finalIndex, double weight, int otherIdx)
        {
            this.otherIdx = otherIdx;
            this.initialIndex = initialIndex;
            this.finalIndex = finalIndex;
            this.weight = weight;
        }

        public override double Evaluate(DoubleVector x)
        {
            int T = x.Length / (KartMPC.xDim + KartMPC.uDim);
            int finalT = T - 1;
            double minDistMine = 1000 * 1000;
            double minDistOther = 1000 * 1000;

            int bestProgressMine = initialIndex;
            int bestProgressOther = initialIndex;

            for (int j = initialIndex; j < finalIndex; j++)
            {
                if (Math.Pow(centerPoints[j % centerPoints.Count].x - x[KartMPC.xIndex * T + (finalT)], 2) + Math.Pow(x[KartMPC.zIndex * T + (finalT)] - centerPoints[j % centerPoints.Count].y, 2) <= minDistMine)
                {
                    minDistMine = Math.Pow(centerPoints[j % centerPoints.Count].x - x[KartMPC.xIndex * T + (finalT)], 2) + Math.Pow(x[KartMPC.zIndex * T + (finalT)] - centerPoints[j % centerPoints.Count].y, 2);
                    bestProgressMine = j;
                }

                if (Math.Pow(centerPoints[j % centerPoints.Count].x - other[KartMPC.xIndex * T + (finalT)], 2) + Math.Pow(other[KartMPC.zIndex * T + (finalT)] - centerPoints[j % centerPoints.Count].y, 2) <= minDistOther)
                {
                    minDistOther = Math.Pow(centerPoints[j % centerPoints.Count].x - other[KartMPC.xIndex * T + (finalT)], 2) + Math.Pow(other[KartMPC.zIndex * T + (finalT)] - centerPoints[j % centerPoints.Count].y, 2);
                    bestProgressOther = j;
                }
            }
            return (-bestProgressMine + bestProgressOther) * weight;
        }

    }
}
