using CenterSpace.NMath.Core;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KartGame.AI.LQR
{
    public abstract class KartLQRCosts : MonoBehaviour
    {
        public abstract DoubleVector getQVec();
        public abstract DoubleMatrix getQMatrix();
    }

    public class LQRCheckpointReachAvoidCost: KartLQRCosts
    {
        DoubleVector target;
        DoubleVector targetWeight;
        Dictionary<KartLQRDynamics, double> reachAvoidWeights;

        public LQRCheckpointReachAvoidCost(DoubleVector targetState, double targetWeight, Dictionary<int, double> avoidWeights, Dictionary<int, int> avoidIndices)
        {

        }

        public override DoubleMatrix getQMatrix()
        {
        }

        public override DoubleVector getQVec()
        {
            var output = new DoubleVector(target);
            output *= targetWeight;

            foreach(KartLQRDynamics a in reachAvoidWeights.Keys)
            {
                output.Append(new DoubleVector(a.getXDim()));
            }

            return output;
        }
    }
}
